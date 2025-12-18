import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import SetRemap, Node, SetParameter

def generate_launch_description():
    # constant's
    current_package_name = 'eureka_simulation'
    nav2_bringup_package_name = 'nav2_bringup'

    # setting's
    use_sim_time = 'true'
    namespace_str = ''

    # config's path
    nav2_params_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'navigation', 'nav.yaml')
    rtabmap_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'navigation', 'rtabmap.yaml')

    # launch's path
    nav2_launch_path = os.path.join(get_package_share_directory(nav2_bringup_package_name), 'launch', 'navigation_launch.py')

    # navigation stack ver.2 start
    nav2 = GroupAction([
        SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments=[('use_sim_time', use_sim_time),
                              ('params_file', nav2_params_path),
                              ('use_composition', 'False'),
                              ('namespace', namespace_str)]
        ),
    ])

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name="rtabmap",
        output="screen",
        # prefix='gdbserver :3000',
        parameters=[rtabmap_config_path],
        remappings=[
            ("/grid_prob_map", "/map"),
            ("scan_cloud", "/camera/points"),
            ("rgb/image", "/camera/image_raw"),
            ("rgb/camera_info", "/camera/camera_info"),
            ("depth/image", "/camera/depth/image_raw")
        ],
        arguments=["--delete_db_on_start"],
    )

    ld = LaunchDescription()
    ld.add_action(nav2)
    ld.add_action(SetParameter(name='use_sim_time', value = True))
    ld.add_action(rtabmap_slam)
    return ld
