"""Launch file for all sensors (cameras, GNSS, and LiDAR)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    """Generate launch description for all sensors."""
    
    # Launch arguments
    vehicle_speed_topic_arg = DeclareLaunchArgument(
        'vehicle_speed_topic',
        default_value='/vehicle/speed',
        description='Topic name for vehicle speed'
    )
    
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='ZED_X',
        description='ZED camera model (ZED_X or ZED2i)'
    )
    
    use_rtk_arg = DeclareLaunchArgument(
        'use_rtk',
        default_value='true',
        description='Use RTK corrections for GNSS if true'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Launch LiDAR if true'
    )
    
    use_lidar_adapter_arg = DeclareLaunchArgument(
        'use_lidar_adapter',
        default_value='true',
        description='Use LiDAR adapter if true'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization if true'
    )
    
    # Include camera launch file
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cameras_launch.py']),
        launch_arguments={
            'vehicle_speed_topic': LaunchConfiguration('vehicle_speed_topic'),
            'camera_model': LaunchConfiguration('camera_model'),
        }.items()
    )
    
    # Include GNSS launch file
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gnss_launch.py']),
        launch_arguments={
            'use_rtk': LaunchConfiguration('use_rtk'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # Include LiDAR launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/lidar_launch.py']),
        launch_arguments={
            'use_adapter': LaunchConfiguration('use_lidar_adapter'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=LaunchConfiguration('use_lidar')
    )
    
    # Return launch description
    return LaunchDescription([
        vehicle_speed_topic_arg,
        camera_model_arg,
        use_rtk_arg,
        use_sim_time_arg,
        use_lidar_arg,
        use_lidar_adapter_arg,
        use_rviz_arg,
        cameras_launch,
        gnss_launch,
        lidar_launch,
    ])