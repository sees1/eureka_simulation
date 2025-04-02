import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

ARGUMENTS = [
    DeclareLaunchArgument('pose_estimator', default_value="'odometry'",
                          choices = ["'eagleye'", "'rgbd_odometry'", "'odometry'"],
                          description='Pose estimators for eureka robot'),
]

def evaluate_spawn(context, *args, **kwargs):
  current_package_name = 'eureka_simulation'
  # eagleye_package_name = 'eagleye_rt'

  pose_estimator = LaunchConfiguration('pose_estimator').perform(context)


  # config's path
  ekf_el_classico_config_path = os.path.join(get_package_share_directory("eureka_simulation"), 'config', 'localization', 'ekf_el_classico.yaml')
  ekf_eagleye_config_path = os.path.join(get_package_share_directory("eureka_simulation"), 'config', 'localization', 'ekf_eagleye_based.yaml')
  ekf_vis_odom_config_path = os.path.join(get_package_share_directory("eureka_simulation"), 'config', 'localization', 'ekf_vis_odom_based.yaml')
  rgbd_config_path = os.path.join(get_package_share_directory("eureka_simulation"), 'config', 'localization', 'rgbd_odometry.yaml')
  # eagleye_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'localization', 'eagleye.yaml')

  # launch's path
  ackermann_launch_path = os.path.join(get_package_share_directory(current_package_name), 'launch', 'controller', 'ackermann.launch.py')
  # eagleye_launch_path = os.path.join(get_package_share_directory(eagleye_package_name), 'launch', 'eagleye_rt.launch.xml')

  # intertial, kinematic: odometry publisher (required)(as pose estimator)
  controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(ackermann_launch_path))
  
  # inertial: pose/twist publisher (optional)(as pose/twist estimator)
  # eagleye_estimator = IncludeLaunchDescription(
  #                         XMLLaunchDescriptionSource(eagleye_launch_path),
  #                         launch_arguments={'config_path': eagleye_config_path}.items(),
  #                         condition = IfCondition(PythonExpression([pose_estimator, " == 'eagleye'"]))
  # )

  # visual: odometry publisher (optional)(as pose/twist estimator)
  rgbd_odometry = Node(
    package="rtabmap_odom",
    executable="rgbd_odometry",
    name="rgbd_odometry",
    output="log",
    parameters=[rgbd_config_path],
    remappings=[
        ("rgb/image", "/camera/image_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("imu", "/imu"),
        ("odom", "/odom_rgbd")
    ],
    arguments=["--ros-args", "--log-level", "warn"],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'rgbd_odometry'"]))
  )

  # high speed = around 60 kmph
  # medium speed = around 5 kmph
  # low speed = around 0.5 kmph

  # pose and twist fusion (classic odometry + imu) (any speed - low-medium precision)
  ekf_el_classico = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    arguments=["--ros-args", "--log-level", "info"],
    parameters=[ekf_el_classico_config_path],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'odometry'"]))
  )

  # pose and twist fusion (eagleye twist + eagleye pose + imu) (high speed - high-medium precision)
  ekf_eagleye = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    arguments=["--ros-args", "--log-level", "info"],
    parameters=[ekf_eagleye_config_path],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'eagleye'"]))
  )

  # pose and twist fusion (visual pose + imu) (low speed - high precision or high speed - low precision)
  ekf_visual = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    arguments=["--ros-args", "--log-level", "info"],
    parameters=[ekf_vis_odom_config_path],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'rgbd_odometry'"]))
  )

  return [controller, rgbd_odometry, ekf_el_classico, ekf_eagleye, ekf_visual]


def generate_launch_description():

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(OpaqueFunction(function=evaluate_spawn))
  return ld