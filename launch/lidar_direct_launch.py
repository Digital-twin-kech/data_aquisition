import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    # Find package directory
    pkg_dir = get_package_share_directory('data_aquisition')
    
    # Fixed parameters for HAP LiDAR based on reference implementation
    xfer_format = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0       # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type = 0
    frame_id = 'livox_frame'
    cmdline_bd_code = 'livox0000000001'
    
    # Configure paths
    config_dir = os.path.join(pkg_dir, 'config', 'lidar')
    config_path = os.path.join(config_dir, 'HAP_config.json')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'pointcloud_viewer.rviz')
    
    # Set parameters
    livox_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"user_config_path": config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    
    # Use reference Livox driver directly
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_params
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_path],
        output='screen'
    )
    
    # Create launch description
    return LaunchDescription([
        livox_driver,
        rviz_node
    ])