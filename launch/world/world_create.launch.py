import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch.actions import OpaqueFunction

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='eureka',
                          description='Model to use for simulation'),
    DeclareLaunchArgument('spawn_robot', default_value='True',
                          choices=['True', 'False'],
                          description='Spawn the eureka robot model.'),
]

def evaluate_spawn(context, *args, **kwargs):
  current_package_name = 'eureka_simulation'

  model = LaunchConfiguration('model').perform(context)
  spawn_robot = LaunchConfiguration('spawn_robot').perform(context)

  # launch's path
  robot_state_launch_path = os.path.join(get_package_share_directory(current_package_name), 'launch', 'world', 'robot_state.launch.py')
  gz_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

  # config's path
  gazebo_params_file = os.path.join(get_package_share_directory(current_package_name),'config', 'world', 'gazebo_params.yaml')

  # world description path
  gazebo_world_file = os.path.join(get_package_share_directory(current_package_name), 'worlds', 'mars3.world')

  #-------------------------------------------------------------------------------------------------------------------------------------------------------

  # robot state publisher
  robot_state = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_launch_path),
                                         launch_arguments={'model': model}.items()
  )

  #gazebo initialization
  gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch_path),
                                    launch_arguments={'world': gazebo_world_file,
                                                      'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
  )

  spawn_entity = Node(package='gazebo_ros',
                      executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', model,
                                 '-x', '0.0',
                                 '-y', '1.5'],
                      output='screen',
                      condition=IfCondition(PythonExpression([spawn_robot, " == True "]))
  )

  use_sim_time_param = SetParameter(name='use_sim_time', value=True)

  return [robot_state, gazebo, spawn_entity, use_sim_time_param]

def generate_launch_description():

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(OpaqueFunction(function=evaluate_spawn))
  return ld