# File: scripts/play_with_clock.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosgraph_msgs.msg import Clock
import time
import sys


class BagClockPlayer(Node):
    def __init__(self, bag_path, rate, storage_id):
        super().__init__("bag_clock_player")
        self.rate = rate

        # Use the storage plugin specified by the user
        storage_opts = StorageOptions(uri=bag_path, storage_id=storage_id)
        conv_opts = ConverterOptions("", "")
        self.reader = SequentialReader()
        self.reader.open(storage_opts, conv_opts)

        # Publisher for /clock
        self.pub = self.create_publisher(Clock, "/clock", 10)

    def play(self):
        first_ts = None
        start_rt = time.time()

        while self.reader.has_next():
            topic, data, ts = self.reader.read_next()
            if first_ts is None:
                first_ts = ts

            # Compute the wall-clock time to publish this timestamp
            target = start_rt + ((ts - first_ts) * 1e-9) * self.rate
            sleep = target - time.time()
            if sleep > 0.0:
                time.sleep(sleep)

            msg = Clock()
            msg.clock.sec = ts // 1_000_000_000
            msg.clock.nanosec = ts % 1_000_000_000
            self.pub.publish(msg)


def main():
    if len(sys.argv) < 2:
        print("Usage: play_with_clock.py <bag_path> [<rate>] [<storage>]")
        sys.exit(1)

    bag_path = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    storage_id = sys.argv[3] if len(sys.argv) > 3 else "mcap"

    rclpy.init()
    player = BagClockPlayer(bag_path, rate, storage_id)
    player.play()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# File: launch/play_with_clock.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments for bag path, rate, and storage plugin
    bag_arg = DeclareLaunchArgument(
        "bag_path",
        description="Absolute path to your ROS2 bag folder",
        default_value="/path/to/bag",
    )
    rate_arg = DeclareLaunchArgument(
        "play_rate", description="Playback rate", default_value="1.0"
    )
    storage_arg = DeclareLaunchArgument(
        "storage", description="Storage plugin: mcap or sqlite3", default_value="mcap"
    )

    # Retrieve configurations
    bag_path = LaunchConfiguration("bag_path")
    play_rate = LaunchConfiguration("play_rate")
    storage_id = LaunchConfiguration("storage")

    return LaunchDescription(
        [
            bag_arg,
            rate_arg,
            storage_arg,
            # 1) Clock-publisher node
            Node(
                package="detection_package",
                executable="play_with_clock.py",
                name="bag_clock_player",
                output="screen",
                arguments=[bag_path, play_rate, storage_id],
            ),
            # 2) After 1s, start bag playback in loop (pause/resume with space)
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "bag",
                            "play",
                            "-i",
                            bag_path,
                            "-s",
                            storage_id,
                            "--rate",
                            play_rate,
                            "--loop",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
