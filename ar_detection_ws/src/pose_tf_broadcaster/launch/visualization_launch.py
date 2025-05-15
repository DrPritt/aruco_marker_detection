from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Broadcast OptiTrack camera pose -> TF
        Node(
            package='pose_tf_broadcaster',
            executable='pose_tf_broadcaster',
            name='optitrack_camera_tf',
            output='screen',
            parameters=[{
                'pose_topic': '/vrpn_client_node/camera/pose',
                'child_frame': 'optitrack_camera'
            }]
        ),

        # Broadcast OptiTrack marker pose -> TF
        Node(
            package='pose_tf_broadcaster',
            executable='pose_tf_broadcaster',
            name='optitrack_marker_tf',
            output='screen',
            parameters=[{
                'pose_topic': '/vrpn_client_node/marker77/pose',
                'child_frame': 'marker_77_optitrack'
            }]
        ),

        # Static transform: world -> camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_camera',
            output='screen',
            arguments=['0', '0', '1', '0', '0', '0', 'optitrack_world', 'camera_link']
        ),

        # Broadcast ArUco marker from camera -> TF (dynamic broadcaster)
        Node(
            package='pose_tf_broadcaster',
            executable='aruco_tf_broadcaster',
            name='aruco_marker_tf',
            output='screen',
            parameters=[{
                'aruco_topic': '/marker_publisher/markers',
                'camera_frame': 'camera_link',
                'child_frame_prefix': 'marker'
            }]
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/piron/ros2_ws/src/your_package_name/rviz/tf_marker_view.rviz']
        )
    ])

