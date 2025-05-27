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

        # Use the storage plugin the user specified:
        storage_opts = StorageOptions(uri=bag_path, storage_id=storage_id)
        conv_opts = ConverterOptions("", "")
        self.reader = SequentialReader()
        self.reader.open(storage_opts, conv_opts)

        # /clock filtering not needed — we use timestamps directly
        self.pub = self.create_publisher(Clock, "/clock", 10)

    def play(self):
        first_ts = None
        start_rt = time.time()

        while self.reader.has_next():
            topic, data, ts = self.reader.read_next()
            if first_ts is None:
                first_ts = ts

            # compute when, in wall-clock, to publish this timestamp:
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
    player.pla
