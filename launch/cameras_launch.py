"""Launch file for ZED Camera nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for cameras."""
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
    
    camera_min_fps_arg = DeclareLaunchArgument(
        'camera_min_fps',
        default_value='15.0',
        description='Minimum camera frame rate'
    )
    
    camera_max_fps_arg = DeclareLaunchArgument(
        'camera_max_fps',
        default_value='60.0',
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
    
    # ZED Camera USB node
    zed_usb_camera_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='zed_2i',  # USB connected ZED 2i camera
        output='screen',
        parameters=[{
            'camera.model': LaunchConfiguration('camera_model'),
            'camera.min_fps': LaunchConfiguration('camera_min_fps'),
            'camera.max_fps': LaunchConfiguration('camera_max_fps'),
            'camera.reliable_qos': LaunchConfiguration('reliable_qos'),
            'camera.qos_history_depth': LaunchConfiguration('qos_history_depth'),
            'camera.pid_p': 0.8,
            'camera.pid_i': 0.2,
            'camera.pid_d': 0.05,
            'camera.serial_number': 37503998,  # USB-0
        }],
        remappings=[
            ('/vehicle/speed', LaunchConfiguration('vehicle_speed_topic')),
        ]
    )

    # ZED Camera GMSL-0 node
    zed_gmsl0_camera_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='zed_x0',  # GMSL-0 connected ZED X camera
        output='screen',
        parameters=[{
            'camera.model': LaunchConfiguration('camera_model'),
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

    # ZED Camera GMSL-1 node
    zed_gmsl1_camera_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='zed_x1',  # GMSL-1 connected ZED X camera
        output='screen',
        parameters=[{
            'camera.model': LaunchConfiguration('camera_model'),
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
    
    # Static transform publishers for camera frames
    zed_2i_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_2i_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'zed_2i_camera_link'],
        output='screen'
    )
    
    zed_x0_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_x0_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'zed_x0_camera_link'],
        output='screen'
    )
    
    zed_x1_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_x1_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'zed_x1_camera_link'],
        output='screen'
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
        camera_model_arg,
        camera_min_fps_arg,
        camera_max_fps_arg,
        reliable_qos_arg,
        qos_history_depth_arg,
        zed_usb_camera_node,
        zed_gmsl0_camera_node,
        zed_gmsl1_camera_node,
        zed_2i_tf_publisher,
        zed_x0_tf_publisher,
        zed_x1_tf_publisher,
        # Comment out RViz node if not needed
        # rviz_node,
    ])