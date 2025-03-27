import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Find package directory
    pkg_dir = get_package_share_directory('data_aquisition')
    
    # Launch arguments
    use_adapter = LaunchConfiguration('use_adapter')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sync = LaunchConfiguration('use_sync')
    
    # Configure paths
    config_path = os.path.join(pkg_dir, 'config', 'lidar', 'livox_params.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'pointcloud_viewer.rviz')
    
    # Declare launch arguments
    declare_use_adapter = DeclareLaunchArgument(
        'use_adapter',
        default_value='true',
        description='Whether to use the Livox adapter node (true) or direct integration (false)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    declare_use_sync = DeclareLaunchArgument(
        'use_sync',
        default_value='true',
        description='Whether to use the synchronization node'
    )
    
    # Create nodes
    livox_node = Node(
        package='data_aquisition',
        executable='livox_lidar_node',
        name='livox_lidar_node',
        namespace='/livox_lidar',
        parameters=[config_path],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    adapter_node = Node(
        package='data_aquisition',
        executable='livox_adapter_node',
        name='livox_adapter_node',
        namespace='/livox_lidar',
        parameters=[{
            'input_point_cloud_topic': '/livox/lidar',
            'input_imu_topic': '/livox/imu',
            'output_point_cloud_topic': 'point_cloud',
            'output_imu_topic': 'imu',
            'frame_id': 'livox_frame',
            'filter_points': True,
            'min_distance': 0.1,
            'max_distance': 100.0,
            'downsample_factor': 1
        }],
        output='screen',
        emulate_tty=True,
        condition=LaunchConfiguration('use_adapter')
    )
    
    # Add static transform publisher for livox_frame to map
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame'],
        output='screen'
    )
    
    # Include the synchronization launch file if needed
    sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'sync_launch.py')]),
        launch_arguments={
            'sync_policy': 'ApproximateTime',
            'time_tolerance': '0.1',
            'cache_size': '20',
            'pass_through': 'true'
        }.items(),
        condition=LaunchConfiguration('use_sync')
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=LaunchConfiguration('use_rviz')
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        declare_use_adapter,
        declare_use_rviz,
        declare_use_sync,
        
        # Nodes
        livox_node,
        adapter_node,
        tf_publisher,
        sync_launch,
        rviz_node
    ])