from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch the live_error_plot node
            Node(
                package="detection_package",
                executable="live_error_plot.py",
                name="live_error_plot",
                output="screen",
            ),
            # Launch rqt_plot to visualize the error topics
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_plot",
                    "rqt_plot",
                    "/error/x:y:z",
                    "/error/distance",
                ],
                shell=True,
            ),
        ]
    )
