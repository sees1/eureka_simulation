import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

import xacro

# normal imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# OpaqueFunction import
from launch.actions import OpaqueFunction


# I declare a function
# it must have context, *args, **kwargs even though I can not seem to access contect or args or kwargs
# the name of the function is used in the call to OpaqueFunction see below
def evaluate_spawn(context, *args, **kwargs):

  model = LaunchConfiguration('model').perform(context)

  pkg_path = os.path.join(get_package_share_directory('eureka_simulation'))
  xacro_file = os.path.join(pkg_path, 'description', model + '.urdf.xacro')

  robot_description_config = Command(['xacro ', xacro_file])

  robot_state_publisher_params = {'robot_description': robot_description_config, 'use_sim_time': True}

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[robot_state_publisher_params]
  )

  return [robot_state_publisher]

def generate_launch_description():
  return LaunchDescription([

    DeclareLaunchArgument(
            'model',
            default_value='eureka',
            description='Use simulation (Gazebo) clock if true'),

    OpaqueFunction(function=evaluate_spawn )  
])