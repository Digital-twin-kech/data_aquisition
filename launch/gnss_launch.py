"""Launch file for GNSS node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for GNSS node."""
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rtk = LaunchConfiguration('use_rtk', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_use_rtk = DeclareLaunchArgument(
        'use_rtk',
        default_value='true',
        description='Use RTK corrections if true'
    )
    
    # GNSS node
    gnss_node = Node(
        package='data_aquisition',
        executable='gnss_node',
        name='gnss_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gnss.model': 'ublox-f9p',
            'gnss.serial_port': '/dev/ttyACM0',
            'gnss.baud_rate': 115200,
            'gnss.frequency': 10.0,
            'gnss.frame_id': 'gnss_frame',
            'gnss.use_rtcm_corrections': use_rtk,
            'gnss.rtcm_source': 'NTRIP',
            'gnss.ntrip_server': 'caster.example.com:2101/RTCM3',
            'gnss.use_dynamic_model': True,
            'gnss.dynamic_model': 'automotive',
            'gnss.qos.reliability': 'reliable',
            'gnss.qos.history_depth': 10
        }],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rtk,
        gnss_node,
    ])