import os

# normal imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='eureka',
                          description='Model to use for simulation')
]

def evaluate_spawn(context, *args, **kwargs):
  current_package_name = 'eureka_simulation'

  model = LaunchConfiguration('model').perform(context)

  # xacro file path
  xacro_file_path = os.path.join(get_package_share_directory(current_package_name), 'description', model + '.urdf.xacro')

  # xacro -> xml convertion
  robot_description_config = Command(['xacro ', xacro_file_path])

  # robot state config with xml
  robot_state_publisher_params = {'robot_description': robot_description_config, 'use_sim_time': True}

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[robot_state_publisher_params]
  )

  return [robot_state_publisher]

def generate_launch_description():

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(OpaqueFunction(function=evaluate_spawn))
  return ld