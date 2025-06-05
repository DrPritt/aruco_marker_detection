#!/usr/bin/env python3

import os
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    import Tkinter as tk
    import ttk

# Directory/File for persisting slider values
SETTINGS_DIR = os.path.expanduser("~/marker_detection")
SETTINGS_FILE = os.path.join(SETTINGS_DIR, "cam_tf_broadcaster.yaml")

# Calibration parameters
CALIBRATION_DURATION_SEC = 2.0  # time window in seconds


class CamTFCalibrator(Node):
    def __init__(self):
        super().__init__("cam_tf_broadcaster")

        # Frames
        self.parent_frame = "world"
        self.camera_frame = "camera"
        self.opti_marker_frame = "marker_aruco_optitrack_raw"
        self.aruco_marker_frame = "marker_aruco_cam_77"

        # TF infrastructure
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Calibration state
        self.start_time = self.get_clock().now()
        self.calibrated = False

        # Load previously saved slider values if available
        self._load_saved_values()

        # Build the GUI (sliders + entry boxes)
        self.root = tk.Tk()
        self.root.title("Camera TF Calibrator & Slider")

        self._make_controls()
        self._apply_loaded_values()

        # Timer at 10 Hz: handle calibration and then slider-based broadcasting
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _make_controls(self):
        """
        Build six slider + entry controls for X, Y, Z, Roll, Pitch, Yaw.
        """
        container = tk.Frame(self.root, padx=10, pady=10)
        container.pack(fill=tk.BOTH, expand=True)

        # Configuration: (label, min, max, resolution)
        self.controls_config = [
            ("X (m)", -100.0, 100.0, 0.001),
            ("Y (m)", -100.0, 100.0, 0.001),
            ("Z (m)", -100.0, 100.0, 0.001),
            ("Roll (rad)", -6.2832, 6.2832, 0.0001),
            ("Pitch (rad)", -6.2832, 6.2832, 0.0001),
            ("Yaw (rad)", -6.2832, 6.2832, 0.0001),
        ]

        self.vars = []  # tk.DoubleVar for sliders
        self.entries = []  # tk.StringVar for entry boxes

        for i, (label, minv, maxv, res) in enumerate(self.controls_config):
            var = tk.DoubleVar()
            tk.Label(container, text=label).grid(row=i, column=0, sticky="w")

            # Slider
            slider = tk.Scale(
                container,
                variable=var,
                from_=minv,
                to=maxv,
                resolution=res,
                orient=tk.HORIZONTAL,
                length=350,
                showvalue=0,
                command=lambda val, idx=i: self._on_slider_change(idx, val),
            )
            slider.grid(row=i, column=1, padx=5, pady=5)

            # Entry box for direct numeric input
            entry_var = tk.StringVar()
            entry = ttk.Entry(container, textvariable=entry_var, width=10)
            entry.grid(row=i, column=2, padx=5, pady=5)
            entry.bind("<Return>", lambda event, idx=i: self._on_entry_change(idx))
            entry.bind("<FocusOut>", lambda event, idx=i: self._on_entry_change(idx))

            self.vars.append(var)
            self.entries.append(entry_var)

        (
            self.x_var,
            self.y_var,
            self.z_var,
            self.roll_var,
            self.pitch_var,
            self.yaw_var,
        ) = self.vars

    def _on_slider_change(self, idx, val):
        """
        When a slider moves, update its entry box to match.
        """
        try:
            v = float(val)
            self.entries[idx].set(f"{v:.6f}")
        except ValueError:
            pass

    def _on_entry_change(self, idx):
        """
        When the user edits the entry box and presses Enter or leaves focus,
        clamp the value to slider range and update the slider.
        """
        try:
            val = float(self.entries[idx].get())
        except ValueError:
            # Reset entry to current slider value
            self.entries[idx].set(f"{self.vars[idx].get():.6f}")
            return

        minv = self.controls_config[idx][1]
        maxv = self.controls_config[idx][2]

        # Clamp to range
        if val < minv:
            val = minv
        elif val > maxv:
            val = maxv

        self.vars[idx].set(val)
        self.entries[idx].set(f"{val:.6f}")

    def _load_saved_values(self):
        """
        Load saved slider values from YAML if available.
        """
        try:
            if not os.path.exists(SETTINGS_DIR):
                os.makedirs(SETTINGS_DIR)
            with open(SETTINGS_FILE, "r") as f:
                data = yaml.safe_load(f)
                if isinstance(data, dict):
                    self.saved_values = data
                else:
                    self.saved_values = {}
        except Exception:
            self.saved_values = {}

    def _apply_loaded_values(self):
        """
        Apply loaded values to sliders and entries (or zeros if none).
        """
        keys = ["x", "y", "z", "roll", "pitch", "yaw"]
        for var, entry_var, key in zip(self.vars, self.entries, keys):
            val = self.saved_values.get(key, 0.0)
            var.set(float(val))
            entry_var.set(f"{float(val):.6f}")

    def _save_slider_values(self):
        """
        Save current slider values to YAML file.
        """
        data = {
            "x": self.x_var.get(),
            "y": self.y_var.get(),
            "z": self.z_var.get(),
            "roll": self.roll_var.get(),
            "pitch": self.pitch_var.get(),
            "yaw": self.yaw_var.get(),
        }
        try:
            with open(SETTINGS_FILE, "w") as f:
                yaml.dump(data, f)
        except Exception as e:
            self.get_logger().warn(f"Failed to save settings: {e}")

    def _timer_callback(self):
        """
        Called at 10 Hz. If not yet calibrated and ≥2 s have passed,
        perform calibration (compute camera pose from OptiTrack & camera detections).
        Then set sliders to that pose and broadcast accordingly.
        After calibration, simply broadcast based on slider values.
        """
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if not self.calibrated and elapsed >= CALIBRATION_DURATION_SEC:
            try:
                # Lookup world → OptiTrack_marker
                opti_tf = self.tf_buffer.lookup_transform(
                    self.parent_frame, self.opti_marker_frame, rclpy.time.Time()
                )
                # Lookup camera → ArUco_marker
                aruco_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, self.aruco_marker_frame, rclpy.time.Time()
                )

                # Build matrix for camera→marker (aruco_tf)
                trans_a = aruco_tf.transform.translation
                rot_a = aruco_tf.transform.rotation
                mat_aruco = tf_transformations.quaternion_matrix(
                    [rot_a.x, rot_a.y, rot_a.z, rot_a.w]
                )
                mat_aruco[0:3, 3] = [trans_a.x, trans_a.y, trans_a.z]
                mat_aruco_inv = tf_transformations.inverse_matrix(mat_aruco)

                # Build matrix for world→marker (opti_tf)
                trans_o = opti_tf.transform.translation
                rot_o = opti_tf.transform.rotation
                mat_opti = tf_transformations.quaternion_matrix(
                    [rot_o.x, rot_o.y, rot_o.z, rot_o.w]
                )
                mat_opti[0:3, 3] = [trans_o.x, trans_o.y, trans_o.z]

                # world→camera = world→marker * (camera→marker)⁻¹
                mat_world_cam = tf_transformations.concatenate_matrices(
                    mat_opti, mat_aruco_inv
                )

                # Extract translation & rotation
                cam_trans = tf_transformations.translation_from_matrix(mat_world_cam)
                cam_quat = tf_transformations.quaternion_from_matrix(mat_world_cam)

                # Update sliders & entries to the computed camera pose
                self.x_var.set(cam_trans[0])
                self.entries[0].set(f"{cam_trans[0]:.6f}")
                self.y_var.set(cam_trans[1])
                self.entries[1].set(f"{cam_trans[1]:.6f}")
                self.z_var.set(cam_trans[2])
                self.entries[2].set(f"{cam_trans[2]:.6f}")

                # Convert quaternion back to RPY
                rpy = tf_transformations.euler_from_quaternion(cam_quat)
                self.roll_var.set(rpy[0])
                self.entries[3].set(f"{rpy[0]:.6f}")
                self.pitch_var.set(rpy[1])
                self.entries[4].set(f"{rpy[1]:.6f}")
                self.yaw_var.set(rpy[2])
                self.entries[5].set(f"{rpy[2]:.6f}")

                self.get_logger().info(
                    f"Calibration complete. Camera pose set to X={cam_trans[0]:.6f}, "
                    f"Y={cam_trans[1]:.6f}, Z={cam_trans[2]:.6f}, "
                    f"Roll={rpy[0]:.6f}, Pitch={rpy[1]:.6f}, Yaw={rpy[2]:.6f}"
                )

                self.calibrated = True

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Calibration TF lookup failed: {e}")
                # Will retry on next timer tick
                return

        # Once calibrated (or if already calibrated), broadcast world→camera from sliders
        if self.calibrated:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            roll = self.roll_var.get()
            pitch = self.pitch_var.get()
            yaw = self.yaw_var.get()

            q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.camera_frame

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z

            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.broadcaster.sendTransform(t)

    def _on_closing(self):
        """
        Save slider values and shutdown cleanly.
        """
        self._save_slider_values()
        self.get_logger().info("Shutting down cam_tf_broadcaster.")
        rclpy.shutdown()
        self.root.destroy()

    def spin(self):
        """
        Spin ROS 2 in a background thread, run Tk mainloop in main thread.
        """
        self.get_logger().info("Starting cam_tf_broadcaster (calibration + sliders)")
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CamTFCalibrator()
    node.spin()


if __name__ == "__main__":
    main()
