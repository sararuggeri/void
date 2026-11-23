#!/usr/bin/env python3
import csv
import math
import os
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class GPSTracker:
    """
    Subscribes to the WAM-V NavSatFix topic, converts (lat, lon) into local
    meters (x_m, y_m) relative to the first fix, and reports/logs each step.
    """

    def __init__(
        self,
        node: Node,
        topic: str = "/wamv/sensors/gps/gps/fix",
        csv_filename: str = "boat_trajectory.csv",
        print_to_console: bool = True,
    ):
        self.node = node
        self.print_to_console = print_to_console

        self.lat0 = None
        self.lon0 = None
        self.curr_lat = None
        self.curr_lon = None
        self.pos_x_m = 0.0
        self.pos_y_m = 0.0

        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), csv_filename)
        self._csv_file = open(log_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ["sim_time_s", "x_m", "y_m", "lat", "lon", "fused_dist_m", "mode"]
        )

        self.node.create_subscription(
            NavSatFix,
            topic,
            self._gps_callback,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info(f"GPSTracker listening on {topic} -> logging to {log_path}")

    def _gps_callback(self, msg: NavSatFix):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return

        self.curr_lat = float(msg.latitude)
        self.curr_lon = float(msg.longitude)

        if self.lat0 is None or self.lon0 is None:
            self.lat0 = self.curr_lat
            self.lon0 = self.curr_lon

        r_earth = 6378137.0
        self.pos_x_m = (
            math.radians(self.curr_lon - self.lon0)
            * r_earth
            * math.cos(math.radians(self.lat0))
        )
        self.pos_y_m = math.radians(self.curr_lat - self.lat0) * r_earth

    def report_step(self, sim_time_s: float, mode: str, fused_dist_m: float = float("nan")):
        if self.curr_lat is None or self.curr_lon is None:
            if self.print_to_console:
                self.node.get_logger().info(f"t={sim_time_s:.2f}s | mode={mode} | GPS=waiting for fix")
            return

        if self.print_to_console:
            self.node.get_logger().info(
                f"t={sim_time_s:.2f}s | mode={mode} | "
                f"pos=({self.pos_x_m:.2f}m, {self.pos_y_m:.2f}m) | "
                f"lat={self.curr_lat:.7f}, lon={self.curr_lon:.7f}"
            )

        self._csv_writer.writerow(
            [
                f"{sim_time_s:.3f}",
                f"{self.pos_x_m:.3f}",
                f"{self.pos_y_m:.3f}",
                f"{self.curr_lat:.7f}",
                f"{self.curr_lon:.7f}",
                f"{fused_dist_m:.3f}" if math.isfinite(fused_dist_m) else "NaN",
                mode,
            ]
        )
        self._csv_file.flush()

    def close(self):
        try:
            self._csv_file.close()
        except Exception:
            pass