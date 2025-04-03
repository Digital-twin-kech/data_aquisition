import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('data_aquisition')
    
    # Synchronization arguments
    sync_policy_arg = DeclareLaunchArgument(
        'sync_policy',
        default_value='ApproximateTime',
        description='Synchronization policy (ExactTime or ApproximateTime)'
    )
    
    time_tolerance_arg = DeclareLaunchArgument(
        'time_tolerance',
        default_value='0.10',
        description='Time tolerance for synchronization in seconds'
    )
    
    cache_size_arg = DeclareLaunchArgument(
        'cache_size',
        default_value='100',
        description='Cache size for synchronization'
    )
    
    max_delay_arg = DeclareLaunchArgument(
        'max_delay',
        default_value='0.5',
        description='Maximum allowable delay for synchronization in seconds'
    )
    
    pass_through_arg = DeclareLaunchArgument(
        'pass_through',
        default_value='true',
        description='Enable direct pass-through of messages without waiting for synchronization'
    )
    
    # Recording arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/records',
        description='Directory to store recorded data'
    )
    
    # Create the nodes
    sync_node = Node(
        package='data_aquisition',
        executable='sync_node',
        name='sensor_synchronizer',
        parameters=[{
            'sync_policy': LaunchConfiguration('sync_policy'),
            'time_tolerance': LaunchConfiguration('time_tolerance'),
            'cache_size': LaunchConfiguration('cache_size'),
            'max_delay': LaunchConfiguration('max_delay'),
            'pass_through': LaunchConfiguration('pass_through'),
            'camera_names': ['ZED_CAMERA_2i', 'ZED_CAMERA_X0', 'ZED_CAMERA_X1'],
            'sync_lidar': True,
            'sync_gnss': True
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'sensor_synchronizer:=info']
    )
    
    data_recorder_node = Node(
        package='data_aquisition',
        executable='data_recorder_node',
        name='data_recorder',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Rosbag recorder has been removed, only using data_recorder_node directly

    return LaunchDescription([
        # Launch arguments
        sync_policy_arg,
        time_tolerance_arg,
        cache_size_arg,
        max_delay_arg,
        pass_through_arg,
        output_dir_arg,
        
        # Nodes
        sync_node,
        data_recorder_node
    ])