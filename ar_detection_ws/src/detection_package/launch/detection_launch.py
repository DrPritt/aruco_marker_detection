import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('video_device', default_value='/dev/video0', description='Video device path'),
        DeclareLaunchArgument('image_width', default_value='640', description='Width of the image'),
        DeclareLaunchArgument('image_height', default_value='480', description='Height of the image'),
        DeclareLaunchArgument('pixel_format', default_value='yuyv', description='Pixel format for the camera'),
        DeclareLaunchArgument('camera_name', default_value='v4l2_camera', description='Camera name'),
        DeclareLaunchArgument('frame_id', default_value='camera_link', description='Frame id for the camera'),
        DeclareLaunchArgument('camera_info_path', default_value='file:///home/piron/camera_calibration/c922_pro_stream_webcam/ost.yaml', description='Path to the camera calibration YAML file'),
	DeclareLaunchArgument('marker_size', default_value='0.08', description='The size of the ArUco marker in meters'),

        # Camera Node (v4l2_camera)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_name': 'v4l2_camera',
                'frame_id': 'camera_link',
                'camera_info_url': LaunchConfiguration('camera_info_path')  # Correctly pass the file path here
            }]
        ),

        # ArUco Marker Detection Node (marker_publisher)
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen',
            parameters=[{
                'markerId': 26,
                'markerSize': LaunchConfiguration('marker_size'),
                'eye': 'right',
                'ref_frame': '/base_link',
            }],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),  # Ensure the topic is remapped properly
            ]
        ),
        
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('launch_complete', 'true'),
            msg="Launching ArUco marker detection and camera nodes!"
        ),
    ])

