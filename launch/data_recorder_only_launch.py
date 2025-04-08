import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('data_aquisition')
    
    # Recording arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/records',
        description='Directory to store recorded data'
    )
    
    # Log level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the data recorder node (debug, info, warn, error)'
    )
    
    # Create the data recorder node only
    data_recorder_node = Node(
        package='data_aquisition',
        executable='data_recorder_node',
        name='data_recorder',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir')
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        # Launch arguments
        output_dir_arg,
        log_level_arg,
        
        # Nodes
        data_recorder_node
    ])