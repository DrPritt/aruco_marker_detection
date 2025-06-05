from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import math

def degrees_to_radians(deg):
    return str(math.radians(float(deg)))

def launch_setup(context, *args, **kwargs):
    # Convert degrees launch arguments to radians strings
    roll_cam = degrees_to_radians(context.launch_configurations['roll_cam'])
    pitch_cam = degrees_to_radians(context.launch_configurations['pitch_cam'])
    yaw_cam = degrees_to_radians(context.launch_configurations['yaw_cam'])
    x_cam = context.launch_configurations['x_cam']
    y_cam = context.launch_configurations['y_cam']
    z_cam = context.launch_configurations['z_cam']

    roll_marker = degrees_to_radians(context.launch_configurations['roll_marker'])
    pitch_marker = degrees_to_radians(context.launch_configurations['pitch_marker'])
    yaw_marker = degrees_to_radians(context.launch_configurations['yaw_marker'])
    x_marker = context.launch_configurations['x_marker']
    y_marker = context.launch_configurations['y_marker']
    z_marker = context.launch_configurations['z_marker']

    return [
        # Publish raw OptiTrack camera pose (ground truth raw)
        Node(
            package="pose_tf_broadcaster",
            executable="optitrack_tf_broadcaster",
            name="optitrack_camera_raw_tf",
            output="screen",
            parameters=[
                {
                    "pose_topic": "/vrpn_mocap/USB_cam/pose",
                    "parent_frame": "world",
                    "child_frame": "cam_actual",
                }
            ],
        ),
        # Static transform publisher: raw camera → corrected camera (apply offsets)
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="camera_offset",
        #     arguments=[
        #         x_cam,
        #         y_cam,
        #         z_cam,
        #         roll_cam,
        #         pitch_cam,
        #         yaw_cam,
        #         "optitrack_camera_raw",
        #         "camera",
        #     ],
        # ),
        # Publish raw OptiTrack marker pose (ground truth raw)
        
        Node(
            package="pose_tf_broadcaster",
            executable="optitrack_tf_broadcaster",
            name="optitrack_marker_raw_tf",
            output="screen",
            parameters=[
                {
                    "pose_topic": "/vrpn_mocap/AR_marker/pose",
                    "parent_frame": "world",
                    "child_frame": "marker_aruco_optitrack_raw",
                }
            ],
        ),
        # Static transform publisher: raw marker → corrected marker (apply offsets)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_offset",
            arguments=[
                x_marker,
                y_marker,
                z_marker,
                roll_marker,
                pitch_marker,
                yaw_marker,
                "marker_aruco_optitrack_raw",
                "ar_marker_optitrack",
            ],
        ),
        
        Node(
            package="pose_tf_broadcaster",  # package containing aruco_tf_broadcaster.py
            executable="aruco_tf_broadcaster",  # must match your setup.py entry
            name="aruco_marker_tf",
            output="screen",
            parameters=[
                {
                    "aruco_topic": "/marker_publisher/markers",
                    "camera_frame": "camera",  # must match the “static_transform_publisher” parent
                    "child_frame_prefix": "marker_aruco_cam_",
                }
            ],
        ),
        
        Node(
            package="pose_tf_broadcaster",  # Use your actual package name
            executable="cam_tf_broadcaster",  # Should match entry in setup.py
            name="cam_tf_broadcaster",
            output="screen",
        ),
    ]

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('roll_cam', default_value='0.0', description='Roll offset for camera frame [deg]'),
        DeclareLaunchArgument('pitch_cam', default_value='0.0', description='Pitch offset for camera frame [deg]'),
        DeclareLaunchArgument('yaw_cam', default_value='0.0', description='Yaw offset for camera frame [deg]'),
        DeclareLaunchArgument('x_cam', default_value='0.0', description='X offset for camera frame [m]'),
        DeclareLaunchArgument('y_cam', default_value='0.0', description='Y offset for camera frame [m]'),
        DeclareLaunchArgument('z_cam', default_value='0.0', description='Z offset for camera frame [m]'),

        DeclareLaunchArgument('roll_marker', default_value='0.0', description='Roll offset for marker frame [deg]'),
        DeclareLaunchArgument('pitch_marker', default_value='0.0', description='Pitch offset for marker frame [deg]'),
        DeclareLaunchArgument('yaw_marker', default_value='0.0', description='Yaw offset for marker frame [deg]'),
        DeclareLaunchArgument('x_marker', default_value='0.0', description='X offset for marker frame [m]'),
        DeclareLaunchArgument('y_marker', default_value='0.0', description='Y offset for marker frame [m]'),
        DeclareLaunchArgument('z_marker', default_value='0.0', description='Z offset for marker frame [m]'),

        OpaqueFunction(function=launch_setup)
    ])
