import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo as NewLogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to the vrpn_mocap client launch YAML for ROS 2 Jazzy
    vrpn_launch_file = os.path.join(
        "/opt/ros/jazzy/share/vrpn_mocap/launch", "client.launch.yaml"
    )

    viz_launch_path = os.path.join(
        get_package_share_directory("pose_tf_broadcaster"),
        "launch",
        "visualization_launch.py",
    )

    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(viz_launch_path),
        launch_arguments={
            # camera offsets
            "roll_cam": "0.0",
            "pitch_cam": "0.0",
            "yaw_cam": "0.0",
            "x_cam": "0.0",
            "y_cam": "0.0",
            "z_cam": "0.0",
            # marker offsets
            "roll_marker": "0.0",
            "pitch_marker": "0.0",
            "yaw_marker": "0.0",
            "x_marker": "0.0",
            "y_marker": "0.0",
            "z_marker": "0.0",
        }.items(),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video4",
                description="Video device path",
            ),
            DeclareLaunchArgument(
                "image_width", default_value="1280", description="Width of the image"
            ),
            DeclareLaunchArgument(
                "image_height", default_value="720", description="Height of the image"
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="YUYV",
                description="Pixel format for the camera",
            ),
            DeclareLaunchArgument(
                "camera_name", default_value="v4l2_camera", description="Camera name"
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera",
                description="Frame id for the camera",
            ),
            DeclareLaunchArgument(
                "camera_info_path",
                default_value="file:///home/ron/marker_detection/camera_calibration/c922_pro_stream_webcam/ost.yaml",
                description="Path to the camera calibration YAML file",
            ),
            DeclareLaunchArgument(
                "marker_size",
                default_value="0.12",
                description="The size of the ArUco marker in meters",
            ),
            # Camera Node
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                output="screen",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "pixel_format": LaunchConfiguration("pixel_format"),
                        "camera_name": LaunchConfiguration("camera_name"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "camera_info_url": LaunchConfiguration("camera_info_path"),
                        # "use_sim_time": True,
                    }
                ],
            ),
            # ArUco Marker Detection Node
            Node(
                package="aruco_ros",
                executable="marker_publisher",
                name="marker_publisher",
                output="screen",
                parameters=[
                    {
                        "marker_id": 77,
                        "marker_size": LaunchConfiguration("marker_size"),
                        "eye": "right",
                        "ref_frame": "camera",
                        # "use_sim_time": True,
                    }
                ],
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
            ),
            Node(
                package="detection_package",
                executable="pose_in_world.py",
                name="pose_in_world",
                output="screen",
            ),
            # Include vrpn_mocap launch file
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(vrpn_launch_file),
                launch_arguments={
                    "server": "192.168.1.134",
                    "update_freq": "60.",
                    # "use_sim_time": "true",
                }.items(),
            ),
            viz_launch,
            NewLogInfo(
                msg="Launching ArUco marker detection, camera, and VRPN client!"
            ),
        ]
    )
