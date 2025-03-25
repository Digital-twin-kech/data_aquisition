from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """
    Launch file for multiple ZED cameras.
    Each camera runs as its own ROS2 node with a distinct node name.
    """
    # Camera serial numbers
    zed2i_serial = 37503998  # ZED 2i (USB) - often not available in this system
    zedx0_serial = 40894785  # ZED X0 (GMSL-0)
    zedx1_serial = 41050786  # ZED X1 (GMSL-1)
    
    # Launch arguments
    camera_resolution_arg = DeclareLaunchArgument(
        'camera_resolution',
        default_value='HD1080',
        description='Camera resolution (HD1080, HD720, VGA)'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15.0',
        description='Camera frame rate'
    )
    
    # ZED 2i Camera Node (USB)
    zed2i_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='ZED_CAMERA_2i',
        namespace='ZED_CAMERA_2i',
        output='screen',
        parameters=[{
            'camera.model': 'ZED2i',
            'camera.resolution': LaunchConfiguration('camera_resolution'),
            'camera.min_fps': LaunchConfiguration('camera_fps'),
            'camera.max_fps': LaunchConfiguration('camera_fps'),
            'camera.serial_number': zed2i_serial,
        }]
    )
    
    # ZED X0 Camera Node (GMSL-0)
    zedx0_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='ZED_CAMERA_X0',
        namespace='ZED_CAMERA_X0',
        output='screen',
        parameters=[{
            'camera.model': 'ZED_X',
            'camera.resolution': LaunchConfiguration('camera_resolution'),
            'camera.min_fps': LaunchConfiguration('camera_fps'),
            'camera.max_fps': LaunchConfiguration('camera_fps'),
            'camera.serial_number': zedx0_serial,
        }]
    )
    
    # ZED X1 Camera Node (GMSL-1)
    zedx1_node = Node(
        package='data_aquisition',
        executable='zed_camera_node',
        name='ZED_CAMERA_X1',
        namespace='ZED_CAMERA_X1',
        output='screen',
        parameters=[{
            'camera.model': 'ZED_X',
            'camera.resolution': LaunchConfiguration('camera_resolution'),
            'camera.min_fps': LaunchConfiguration('camera_fps'),
            'camera.max_fps': LaunchConfiguration('camera_fps'),
            'camera.serial_number': zedx1_serial,
        }]
    )
    
    return LaunchDescription([
        camera_resolution_arg,
        camera_fps_arg,
        zed2i_node,
        zedx0_node,
        zedx1_node,
    ])