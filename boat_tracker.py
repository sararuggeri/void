#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO


class KalmanDistance1D:
    """
    Filtro di Kalman 1D lineare [distanza_m, velocita_relativa_mps].
    Fonde camera e sonar con pesi 50/50 (tramite varianze di misura) e scarta outlier via gating.
    """

    def __init__(self, initial_dist=5.0, reliability_cam=0.5, reliability_sonar=0.5):
        self.x = np.array([[initial_dist], [0.0]], dtype=np.float64)
        self.P = np.diag([2.0, 1.0]).astype(np.float64)
        self.Q = np.diag([0.04, 0.25]).astype(np.float64)

        total = max(1e-6, reliability_cam + reliability_sonar)
        w_cam = reliability_cam / total
        w_sonar = reliability_sonar / total
        base_r = 0.25
        self.R_cam = base_r / max(0.05, w_cam)
        self.R_sonar = base_r / max(0.05, w_sonar)

        self.H = np.array([[1.0, 0.0]], dtype=np.float64)
        self.I2 = np.eye(2, dtype=np.float64)

    def predict(self, dt: float):
        dt = float(np.clip(dt, 1e-3, 0.5))
        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z: float, R: float, gate_m: float) -> bool:
        pred_dist = float(self.x[0, 0])
        if abs(z - pred_dist) > gate_m:
            return False
        y = float(z) - float((self.H @ self.x)[0, 0])
        S = float((self.H @ self.P @ self.H.T)[0, 0]) + R
        K = (self.P @ self.H.T) / max(1e-9, S)
        self.x = self.x + K * y
        self.P = (self.I2 - K @ self.H) @ self.P
        return True

    def step(self, dt: float, cam_dist=None, sonar_dist=None):
        self.predict(dt)
        used_cam = False
        used_sonar = False
        if cam_dist is not None and np.isfinite(cam_dist):
            used_cam = self.update(float(cam_dist), self.R_cam, gate_m=6.0)
        if sonar_dist is not None and np.isfinite(sonar_dist):
            used_sonar = self.update(float(sonar_dist), self.R_sonar, gate_m=3.0)

        dist = float(np.clip(self.x[0, 0], 0.3, 30.0))
        rel_vel = float(np.clip(self.x[1, 0], -8.0, 8.0))
        return dist, rel_vel, used_cam, used_sonar


class BoatTracker(Node):
    """
    Controllore USV per inseguimento target a ~2.0m con velocita' superiore al nuotatore normale (~1.1 m/s).
    """

    SWIMMER_SPEED_MPS = 1.1
    BOAT_TOP_SPEED_MPS = 2.4
    TARGET_DIST_M = 2.0
    BRAKE_DIST_M = 1.75
    MIN_SAFE_DIST_M = 1.45
    CAM_HFOV_RAD = np.deg2rad(70.0)

    MAX_THRUST_N = 280.0
    MAX_FORWARD_N = 230.0
    MAX_REVERSE_N = -130.0
    MAX_TURN_N = 95.0

    KP_TURN = 78.0
    KD_TURN = 14.0
    KP_DIST = 85.0
    KD_VEL = 28.0
    FF_VEL_GAIN = 32.0

    INFER_WIDTH = 640
    CONF_THRESHOLD = 0.45

    def __init__(self):
        super().__init__('boat_tracker')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.bridge = CvBridge()
        self.latest_sonar_range = None
        self.kf = KalmanDistance1D(initial_dist=5.0, reliability_cam=0.5, reliability_sonar=0.5)

        now_sec = self._now_sec()
        self.last_time_sec = now_sec
        self.prev_norm_err_x = 0.0
        self.filtered_err_rate = 0.0

        self.last_seen_side = 1.0
        self.lost_target_elapsed = 0.0
        self.has_seen_target = False
        self.busy = False

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.quantize_mode = 16 if self.device == 'cuda' else 32
        self.get_logger().info(f"Caricamento YOLOv8 su {self.device.upper()} (quantize={self.quantize_mode})...")
        self.model = YOLO('yolov8n.pt')
        self.model.to(self.device)

        self.create_subscription(
            Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 1
        )
        self.create_subscription(
            LaserScan, '/wamv/sensors/sonar/scan', self.sonar_callback, 5
        )

        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.get_logger().info(
            f"BoatTracker pronto | Target={self.TARGET_DIST_M:.1f}m | TopSpeed={self.BOAT_TOP_SPEED_MPS:.1f}m/s vs Swimmer~{self.SWIMMER_SPEED_MPS:.1f}m/s"
        )

    def _now_sec(self) -> float:
        t = self.get_clock().now().nanoseconds * 1e-9
        return t if t > 0.0 else 0.0

    def sonar_callback(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        min_r = max(float(msg.range_min), 0.25) if np.isfinite(msg.range_min) else 0.25
        max_r = (float(msg.range_max) - 0.10) if np.isfinite(msg.range_max) else 20.0
        mask = np.isfinite(ranges) & (ranges >= min_r) & (ranges <= max_r)
        if np.any(mask):
            self.latest_sonar_range = float(np.min(ranges[mask]))
        else:
            self.latest_sonar_range = None

    def publish_thrust(self, forward_cmd: float, turn_cmd: float):
        eff_turn = turn_cmd if forward_cmd >= -1e-3 else -turn_cmd
        left = float(np.clip(forward_cmd - eff_turn, -self.MAX_THRUST_N, self.MAX_THRUST_N))
        right = float(np.clip(forward_cmd + eff_turn, -self.MAX_THRUST_N, self.MAX_THRUST_N))
        self.left_thrust_pub.publish(Float64(data=left))
        self.right_thrust_pub.publish(Float64(data=right))

    def estimate_cam_distance(self, box_w: float, box_h: float, img_w: float) -> float:
        apparent_size = max(float(box_w), float(box_h), 1.0)
        return float(np.clip((260.0 / apparent_size) * (img_w / 640.0), 0.4, 25.0))

    def _detect_person(self, frame_bgr: np.ndarray):
        h, w, _ = frame_bgr.shape
        if w > self.INFER_WIDTH:
            scale = self.INFER_WIDTH / float(w)
            infer_frame = cv2.resize(frame_bgr, (self.INFER_WIDTH, max(1, int(h * scale))), interpolation=cv2.INTER_LINEAR)
            inv_scale = 1.0 / scale
        else:
            infer_frame = frame_bgr
            inv_scale = 1.0

        ih, iw, _ = infer_frame.shape
        center_x = iw * 0.5

        with torch.inference_mode():
            results = self.model(
                infer_frame,
                classes=[0],
                conf=self.CONF_THRESHOLD,
                quantize=self.quantize_mode,
                verbose=False,
            )

        best = None
        best_score = float('inf')

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            for (x1, y1, x2, y2), conf in zip(boxes, confs):
                cx = 0.5 * (x1 + x2)
                norm_offset = abs(cx - center_x) / max(1.0, center_x)
                score = norm_offset - 0.15 * float(conf)
                if score < best_score:
                    best_score = score
                    best = (x1 * inv_scale, y1 * inv_scale, x2 * inv_scale, y2 * inv_scale, cx * inv_scale, float(conf))

        return best

    def image_callback(self, msg: Image):
        if self.busy:
            return
        self.busy = True

        try:
            now_sec = self._now_sec()
            dt = now_sec - self.last_time_sec
            if dt <= 1e-4 or dt > 1.0:
                dt = 0.05
            self.last_time_sec = now_sec

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, width, _ = cv_image.shape
            img_center_x = width * 0.5

            detection = self._detect_person(cv_image)

            if detection is not None:
                x1, y1, x2, y2, cx, conf = detection
                self.has_seen_target = True
                self.lost_target_elapsed = 0.0

                norm_err_x = float(np.clip((cx - img_center_x) / max(1.0, img_center_x), -1.0, 1.0))
                if abs(norm_err_x) > 0.04:
                    self.last_seen_side = -1.0 if norm_err_x < 0.0 else 1.0

                raw_rate = (norm_err_x - self.prev_norm_err_x) / dt
                self.prev_norm_err_x = norm_err_x
                alpha_d = dt / (0.15 + dt)
                self.filtered_err_rate = (1.0 - alpha_d) * self.filtered_err_rate + alpha_d * raw_rate

                cam_dist = self.estimate_cam_distance(x2 - x1, y2 - y1, width)
                fused_dist, rel_vel, used_cam, used_sonar = self.kf.step(
                    dt, cam_dist=cam_dist, sonar_dist=self.latest_sonar_range
                )

                raw_turn = -self.KP_TURN * norm_err_x - self.KD_TURN * self.filtered_err_rate
                turn_cmd = float(np.clip(raw_turn, -self.MAX_TURN_N, self.MAX_TURN_N))

                dist_err = fused_dist - self.TARGET_DIST_M
                ff_cmd = self.FF_VEL_GAIN * np.clip(rel_vel, -1.0, 4.0)

                if fused_dist < self.MIN_SAFE_DIST_M:
                    penetration = self.MIN_SAFE_DIST_M - fused_dist
                    forward_cmd = float(np.clip(-(75.0 + 140.0 * penetration), self.MAX_REVERSE_N, 0.0))
                    mode_txt = "REV-SAFE"
                elif fused_dist < self.BRAKE_DIST_M:
                    brake_err = self.BRAKE_DIST_M - fused_dist
                    forward_cmd = float(np.clip(-(25.0 + 95.0 * brake_err), self.MAX_REVERSE_N, 0.0))
                    mode_txt = "BRAKE"
                elif dist_err > 0.12:
                    raw_fwd = self.KP_DIST * dist_err + self.KD_VEL * max(0.0, rel_vel) + ff_cmd + 45.0
                    forward_cmd = float(np.clip(raw_fwd, 0.0, self.MAX_FORWARD_N))
                    mode_txt = "CHASE"
                else:
                    forward_cmd = float(np.clip(max(0.0, 0.5 * ff_cmd), 0.0, self.MAX_FORWARD_N * 0.4))
                    mode_txt = "HOLD"

                self.publish_thrust(forward_cmd, turn_cmd)

                sonar_str = f"{self.latest_sonar_range:.2f}m" if self.latest_sonar_range is not None else "N/A"
                src_str = f"C:{int(used_cam)}/S:{int(used_sonar)}"
                overlay = f"person {conf:.2f} | KF:{fused_dist:.2f}m [{mode_txt}]"
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 255), 2)
                cv2.putText(
                    cv_image, overlay, (int(x1), max(25, int(y1) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2
                )
                self.get_logger().info(
                    f"YOLO={conf:.2f} | cam={cam_dist:.2f}m | sonar={sonar_str} ({src_str}) | KF={fused_dist:.2f}m (dv={rel_vel:+.2f}) | {mode_txt} fwd={forward_cmd:.1f}N turn={turn_cmd:.1f}N",
                    throttle_duration_sec=0.8
                )
            else:
                fused_dist, rel_vel, _, used_sonar = self.kf.step(
                    dt, cam_dist=None, sonar_dist=self.latest_sonar_range
                )
                self.lost_target_elapsed += dt
                self.filtered_err_rate = 0.0

                if fused_dist < self.BRAKE_DIST_M:
                    forward_cmd = -45.0
                else:
                    forward_cmd = 0.0

                if self.has_seen_target and self.lost_target_elapsed <= 3.0:
                    side_label = "SX" if self.last_seen_side < 0.0 else "DX"
                    turn_cmd = float(-self.last_seen_side * (self.MAX_TURN_N * 0.85))
                else:
                    side_label = "SCAN"
                    turn_cmd = float(self.MAX_TURN_N * 0.55)

                self.publish_thrust(forward_cmd, turn_cmd)
                cv2.putText(
                    cv_image, f"Perso -> {side_label} | KF:{fused_dist:.2f}m", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 140, 255), 2
                )

            cv2.imshow("Fusion Tracker - YOLO + Kalman (50/50)", cv_image)
            cv2.waitKey(1)

        except Exception as exc:
            self.get_logger().error(f"Errore in image_callback: {exc}")
        finally:
            self.busy = False

    def stop_motors_safely(self):
        try:
            if rclpy.ok():
                self.publish_thrust(0.0, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = BoatTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors_safely()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
