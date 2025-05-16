from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Broadcast camera pose → TF
        Node(
            package='pose_tf_broadcaster',
            executable='optitrack_tf_broadcaster',
            name='optitrack_camera_tf',
            output='screen',
            parameters=[{
                'pose_topic':   '/vrpn_mocap/USB_cam/pose',
                'parent_frame': 'world',
                'child_frame':  'optitrack_camera'
            }]
        ),

        # Broadcast marker pose → TF
        Node(
            package='pose_tf_broadcaster',
            executable='optitrack_tf_broadcaster',
            name='optitrack_marker_tf',
            output='screen',
            parameters=[{
                'pose_topic':   '/vrpn_mocap/AR_marker/pose',
                'parent_frame': 'world',
                'child_frame':  'marker_aruco_optitrack'
            }]
        ),

        # Broadcast ArUco vision markers -> TF
        Node(
            package='pose_tf_broadcaster',
            executable='aruco_tf_broadcaster',
            name='aruco_marker_tf',
            parameters=[{
                'aruco_topic':        '/marker_publisher/markers',
                'camera_frame':       'optitrack_camera',
                'child_frame_prefix': 'marker_aruco_cam_'
            }]
        ),
    ])

