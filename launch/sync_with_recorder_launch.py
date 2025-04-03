import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('data_aquisition')
    
    # Include sync launch file
    sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'sync_launch.py')
        ),
        launch_arguments={
            'sync_policy': LaunchConfiguration('sync_policy'),
            'time_tolerance': LaunchConfiguration('time_tolerance'),
            'pass_through': LaunchConfiguration('pass_through')
        }.items()
    )
    
    # Include data recorder launch file 
    data_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'data_recorder_only_launch.py')
        ),
        launch_arguments={
            'output_dir': LaunchConfiguration('output_directory')
        }.items()
    )
    
    # Create the launch description with arguments
    return LaunchDescription([
        # Sync arguments
        DeclareLaunchArgument(
            'sync_policy',
            default_value='ApproximateTime',
            description='Synchronization policy (ApproximateTime, ExactTime)'
        ),
        
        DeclareLaunchArgument(
            'time_tolerance',
            default_value='0.10',
            description='Time tolerance for synchronization in seconds'
        ),
        
        DeclareLaunchArgument(
            'pass_through',
            default_value='true',
            description='Whether to pass through messages without waiting for synchronization'
        ),
        
        # Recorder arguments
        DeclareLaunchArgument(
            'output_directory',
            default_value='/home/user/rosbags',
            description='Directory to store ROS2 bags'
        ),
        
        DeclareLaunchArgument(
            'auto_record',
            default_value='true',
            description='Auto-activate recording'
        ),
        
        # Include launch files
        sync_launch,
        data_recorder_launch
    ])