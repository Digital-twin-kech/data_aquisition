import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create a node to monitor the synchronized topics
    return LaunchDescription([
        # Camera topics
        Node(
            package='topic_tools',
            executable='hz',
            name='camera_2i_image_hz',
            arguments=['/synchronized/ZED_CAMERA_2i/rgb/image_rect_color'],
            output='screen'
        ),
        
        Node(
            package='topic_tools',
            executable='hz',
            name='camera_2i_imu_hz',
            arguments=['/synchronized/ZED_CAMERA_2i/IMU'],
            output='screen'
        ),
        
        Node(
            package='topic_tools',
            executable='hz',
            name='camera_x0_image_hz',
            arguments=['/synchronized/ZED_CAMERA_X0/rgb/image_rect_color'],
            output='screen'
        ),
        
        Node(
            package='topic_tools',
            executable='hz',
            name='camera_x1_image_hz',
            arguments=['/synchronized/ZED_CAMERA_X1/rgb/image_rect_color'],
            output='screen'
        ),
        
        # LiDAR topic
        Node(
            package='topic_tools',
            executable='hz',
            name='lidar_hz',
            arguments=['/synchronized/livox/lidar'],
            output='screen'
        ),
        
        # GNSS topic
        Node(
            package='topic_tools',
            executable='hz',
            name='gnss_hz',
            arguments=['/synchronized/gnss/fix'],
            output='screen'
        )
    ])