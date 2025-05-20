import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Path to the vrpn_mocap client launch YAML for ROS 2 Jazzy
    vrpn_launch_file = os.path.join(
        '/opt/ros/jazzy/share/vrpn_mocap/launch',
        'client.launch.yaml'
    )

    viz_launch_path = os.path.join(
    get_package_share_directory('pose_tf_broadcaster'),
    'launch',
    'visualization_launch.py'
    )
    
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(viz_launch_path),
        # optionally, propagate arguments:
        # launch_arguments={'roll_cam': LaunchConfiguration('roll_cam'),
        #                   'pitch_cam': LaunchConfiguration('pitch_cam'),
        #                   …}.items()
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('video_device', default_value='/dev/video0', description='Video device path'),
        DeclareLaunchArgument('image_width', default_value='640', description='Width of the image'),
        DeclareLaunchArgument('image_height', default_value='480', description='Height of the image'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv', description='Pixel format for the camera'),
        DeclareLaunchArgument('camera_name', default_value='v4l2_camera', description='Camera name'),
        DeclareLaunchArgument('frame_id', default_value='camera_link', description='Frame id for the camera'),
        DeclareLaunchArgument('camera_info_path', default_value='file:///home/piron/marker_detection/camera_calibration/c922_pro_stream_webcam/ost.yaml', description='Path to the camera calibration YAML file'),
        DeclareLaunchArgument('marker_size', default_value='0.12', description='The size of the ArUco marker in meters'),

        # Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'camera_name': LaunchConfiguration('camera_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_info_url': LaunchConfiguration('camera_info_path')
            }]
        ),

        # ArUco Marker Detection Node
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen',
            parameters=[{
                'marker_id': 77,
                'marker_size': LaunchConfiguration('marker_size'),
                'eye': 'right',
                'ref_frame': 'camera_link',
            }],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
            ]
        ),

        # Include vrpn_mocap launch file
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(vrpn_launch_file),
            launch_arguments={'server': '192.168.1.134'}.items()
        ),
        
        viz_launch,

        LogInfo(msg="Launching ArUco marker detection, camera, and VRPN client!"),
    ])

