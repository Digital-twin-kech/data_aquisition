"""Launch file for ZED X cameras."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for ZED X cameras."""
    # Launch arguments
    vehicle_speed_topic_arg = DeclareLaunchArgument(
        'vehicle_speed_topic',
        default_value='/vehicle/speed',
        description='Topic name for vehicle speed'
    )
    
    camera_resolution_arg = DeclareLaunchArgument(
        'camera_resolution',
        default_value='HD1080',
        description='Camera resolution (HD1080, HD720, VGA)'
    )
    
    camera_min_fps_arg = DeclareLaunchArgument(
        'camera_min_fps',
        default_value='15.0',
        description='Minimum camera frame rate'
    )
    
    camera_max_fps_arg = DeclareLaunchArgument(
        'camera_max_fps',
        default_value='15.0',
        description='Maximum camera frame rate'
    )
    
    reliable_qos_arg = DeclareLaunchArgument(
        'reliable_qos',
        default_value='true',
        description='Use reliable QoS for camera publishers'
    )
    
    qos_history_depth_arg = DeclareLaunchArgument(
        'qos_history_depth',
        default_value='5',
        description='QoS history depth for camera publishers'
    )
    
    # ZED X GMSL-0 camera node
    zed_x0_camera_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='zed_x0',  # GMSL-0 connected ZED X camera
        output='screen',
        parameters=[{
            'camera.model': 'ZED_X',
            'camera.resolution': LaunchConfiguration('camera_resolution'),
            'camera.min_fps': LaunchConfiguration('camera_min_fps'),
            'camera.max_fps': LaunchConfiguration('camera_max_fps'),
            'camera.reliable_qos': LaunchConfiguration('reliable_qos'),
            'camera.qos_history_depth': LaunchConfiguration('qos_history_depth'),
            'camera.pid_p': 0.8,
            'camera.pid_i': 0.2,
            'camera.pid_d': 0.05,
            'camera.serial_number': 40894785,  # GMSL-0
        }],
        remappings=[
            ('/vehicle/speed', LaunchConfiguration('vehicle_speed_topic')),
        ]
    )

    # ZED X GMSL-1 camera node
    zed_x1_camera_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='zed_x1',  # GMSL-1 connected ZED X camera
        output='screen',
        parameters=[{
            'camera.model': 'ZED_X',
            'camera.resolution': LaunchConfiguration('camera_resolution'),
            'camera.min_fps': LaunchConfiguration('camera_min_fps'),
            'camera.max_fps': LaunchConfiguration('camera_max_fps'),
            'camera.reliable_qos': LaunchConfiguration('reliable_qos'),
            'camera.qos_history_depth': LaunchConfiguration('qos_history_depth'),
            'camera.pid_p': 0.8,
            'camera.pid_i': 0.2,
            'camera.pid_d': 0.05,
            'camera.serial_number': 41050786,  # GMSL-1
        }],
        remappings=[
            ('/vehicle/speed', LaunchConfiguration('vehicle_speed_topic')),
        ]
    )
    
    # Optional RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('data_aquisition'),
            'config', 'camera_view.rviz')],
        output='screen'
    )

    return LaunchDescription([
        vehicle_speed_topic_arg,
        camera_resolution_arg,
        camera_min_fps_arg,
        camera_max_fps_arg,
        reliable_qos_arg,
        qos_history_depth_arg,
        zed_x0_camera_node,
        zed_x1_camera_node,
        # Comment out RViz node if not needed
        # rviz_node,
    ])