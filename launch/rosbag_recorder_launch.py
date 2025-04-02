import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('data_aquisition')
    
    # Create the parameters path
    default_params_file = os.path.join(
        package_dir, 
        'config', 
        'recording', 
        'rosbag_recorder_params.yaml'
    )
    
    # Launch Arguments
    params_file = LaunchConfiguration('params_file')
    output_directory = LaunchConfiguration('output_directory')
    auto_activate = LaunchConfiguration('auto_activate')
    
    # Create the Node
    recorder_node = Node(
        package='data_aquisition',
        executable='rosbag_recorder_node',
        name='rosbag_recorder',
        namespace='',
        output='screen',
        parameters=[params_file],
        arguments=['--auto-activate'] if auto_activate == 'true' else [],
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'params_file', 
            default_value=default_params_file,
            description='Path to the ROS2 parameters file to use'
        ),
        
        DeclareLaunchArgument(
            'output_directory',
            default_value='',
            description='Directory to store ROS2 bags'
        ),
        
        DeclareLaunchArgument(
            'auto_activate',
            default_value='true',
            description='Auto-activate the recorder node'
        ),
        
        # The node
        recorder_node
    ])