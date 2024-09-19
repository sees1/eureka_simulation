import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
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
    DeclareLaunchArgument('mapping_mode', default_value='false',
                          choices=['true', 'false'],
                          description='Make map with lidar'),
    DeclareLaunchArgument('localization_mode', default_value='false',
                          choices=['true', 'false'],
                          description='Localize with lidar'),
    DeclareLaunchArgument('navigation_mode', default_value='false',
                          choices=['true', 'false'],
                          description='Make Navigation with lidar'),
    DeclareLaunchArgument('navigation_slam_mode', default_value='false',
                          choices=['true', 'false'],
                          description='Make Navigation SLAM with lidar'),
    DeclareLaunchArgument('model', default_value='eureka',
                          description='Model to use for simulation'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]


def generate_launch_description():

  package_name = 'eureka_simulation'
  navigation_pkg = 'eureka_navigation'

  model_type = LaunchConfiguration('model')

  robot_state = IncludeLaunchDescription(
                      PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name), 'launch', 'robot_state.launch.py')
                      ]), launch_arguments={'model': model_type}.items()
  )

  gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
  gazebo_world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'mars.world')

  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                  launch_arguments={'world': gazebo_world_file, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
  )

  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', model_type,
                                 '-x', '0.0',
                                 '-y', '5.0'],
                      output='screen',
                      condition=IfCondition(LaunchConfiguration('spawn_robot'))
  )

  mapping_node = IncludeLaunchDescription(
                      PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(navigation_pkg), 'launch', 'mapping.launch.py')
                      ]),
                      condition = IfCondition(LaunchConfiguration('mapping_mode'))
  )

  rviz2_config = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'rviz', 'eureka.rviz'])

  rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen'
  )

  localization_node = IncludeLaunchDescription(
                      PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(navigation_pkg), 'launch', 'localization.launch.py')
                      ]),
                      condition = IfCondition(LaunchConfiguration('localization_mode'))
  )

  navigation_load = GroupAction([
    ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "eureka_navigation",
                    "localization.launch.py"
                ]
    ),
    ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "eureka_navigation",
                    "nav2.launch.py"
                ]
    )],
    condition = IfCondition(LaunchConfiguration('navigation_mode'))
  )

  navigation_slam_load = GroupAction([
    ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "eureka_navigation",
                    "nav2tune.launch.py"
                ]
    )],
    condition = IfCondition(LaunchConfiguration('navigation_slam_mode'))
  )

  ack_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    name='ack_drive_spawner',
    arguments=["ack_cont"]
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
  ld.add_action(mapping_node)
  ld.add_action(rviz)
  ld.add_action(localization_node)
  ld.add_action(navigation_load)
  ld.add_action(navigation_slam_load)
  ld.add_action(ack_drive_spawner)
  ld.add_action(joint_broad_spawner)
  return ld