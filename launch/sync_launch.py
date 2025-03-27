from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    sync_policy_arg = DeclareLaunchArgument(
        'sync_policy',
        default_value='ApproximateTime',
        description='Synchronization policy (ExactTime or ApproximateTime)'
    )
    
    time_tolerance_arg = DeclareLaunchArgument(
        'time_tolerance',
        default_value='0.02',
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
    
    # Create sensor synchronizer node
    sync_node = Node(
        package='data_aquisition',
        executable='sync_node',
        name='sensor_synchronizer',
        parameters=[{
            'sync_policy': LaunchConfiguration('sync_policy'),
            'time_tolerance': LaunchConfiguration('time_tolerance'),
            'cache_size': LaunchConfiguration('cache_size'),
            'max_delay': LaunchConfiguration('max_delay'),
            'camera_names': ['ZED_CAMERA_2i', 'ZED_CAMERA_X0', 'ZED_CAMERA_X1'],
            'sync_lidar': True,
            'sync_gnss': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        sync_policy_arg,
        time_tolerance_arg,
        cache_size_arg,
        max_delay_arg,
        sync_node
    ])