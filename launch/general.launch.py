import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Start gzclient.'),
    DeclareLaunchArgument('spawn_robot', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the eureka robot model.'),
    DeclareLaunchArgument('model', default_value='eureka',
                          description='Model to use for simulation'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]


def generate_launch_description():

  package_name = 'eureka_simulation'

  model_type = LaunchConfiguration('model')

  gazebo_launch_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'launch', 'gazebo.launch.py'])

  robot_state = IncludeLaunchDescription(
                      PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name), 'launch', 'robot_state.launch.py')
                      ]), launch_arguments={'model': model_type}.items()
  )

  gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                  launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
           )

  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', model_type],
                      output='screen',
                      condition=IfCondition(LaunchConfiguration('spawn_robot'))
  )

  diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    name='diff_drive_spawner',
    arguments=["diff_cont"]
  )

  joint_broad_spawner = Node(
    package='controller_manager',
    executable='spawner',
    name='joint_broad_spawner',
    arguments=["joint_broad"]
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(robot_state)
  ld.add_action(gazebo)
  ld.add_action(spawn_entity)
  ld.add_action(diff_drive_spawner)
  ld.add_action(joint_broad_spawner)
  return ld