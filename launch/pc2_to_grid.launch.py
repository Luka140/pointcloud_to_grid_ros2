import os
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    default_param_file_path = os.path.join(
        get_package_share_directory('pointcloud_to_grid'),
        'config',
        'default_config.yaml')
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file_path,
        description='Full path to the ROS2 parameters file to use.')
    
    ld = LaunchDescription()
    ld.add_action(param_file_arg)

    ld.add_action(launch_ros.actions.Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            name='ptg',
            output='screen',
            parameters=[LaunchConfiguration('param_file')],
        )
    )

    return ld