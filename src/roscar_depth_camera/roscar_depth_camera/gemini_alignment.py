"""Gemini red-platform visual alignment with fail-closed low-speed output."""

from collections import deque
import json
import math
import time

import cv2
from cv_bridge import CvBridge
from c50c_interfaces.msg import BaseStatus
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from .processing import (
    depth_to_meters,
    detect_red_target,
    stepped_alignment_velocity,
    visual_alignment_velocity,
)


def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class GeminiAlignment(Node):
    def __init__(self):
        super().__init__('gemini_visual_alignment')
        self.declare_parameter('motion_enabled', True)
        self.declare_parameter('reference_center_x', 0.5)
        self.declare_parameter('reference_center_y', 0.5)
        self.declare_parameter('center_tolerance', 0.025)
        self.declare_parameter('vertical_tolerance', 0.035)
        self.declare_parameter('minimum_safe_depth_m', 0.28)
        self.declare_parameter('maximum_runtime_sec', 60.0)
        self.declare_parameter('min_forward_speed', 0.040)
        self.declare_parameter('max_forward_speed', 0.050)
        self.declare_parameter('min_lateral_speed', 0.050)
        self.declare_parameter('max_lateral_speed', 0.060)
        self.declare_parameter('move_step_sec', 0.30)
        self.declare_parameter('settle_step_sec', 0.45)

        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.reference_center_x = float(
            self.get_parameter('reference_center_x').value
        )
        self.reference_center_y = float(
            self.get_parameter('reference_center_y').value
        )
        self.center_tolerance = float(
            self.get_parameter('center_tolerance').value
        )
        self.vertical_tolerance = float(
            self.get_parameter('vertical_tolerance').value
        )
        self.minimum_safe_depth_m = float(
            self.get_parameter('minimum_safe_depth_m').value
        )
        self.maximum_runtime_sec = float(
            self.get_parameter('maximum_runtime_sec').value
        )
        self.min_forward_speed = float(
            self.get_parameter('min_forward_speed').value
        )
        self.max_forward_speed = float(
            self.get_parameter('max_forward_speed').value
        )
        self.min_lateral_speed = float(
            self.get_parameter('min_lateral_speed').value
        )
        self.max_lateral_speed = float(
            self.get_parameter('max_lateral_speed').value
        )
        self.move_step_sec = float(
            self.get_parameter('move_step_sec').value
        )
        self.settle_step_sec = float(
            self.get_parameter('settle_step_sec').value
        )
        self.bridge = CvBridge()
        self.started_at = time.monotonic()
        self.last_image_at = 0.0
        self.last_depth_at = 0.0
        self.base_seen_at = 0.0
        self.safe_cmd_seen_at = 0.0
        self.depth_m = None
        self.base_fault_bits = None
        self.estop = True
        self.goal_active = True
        self.safe_speed = math.inf
        self.motion_started = False
        self.finished = False
        self.aligned_frames = 0
        self.detections = deque(maxlen=6)
        self.step_command = (0.0, 0.0)
        self.move_until = 0.0
        self.settle_until = 0.0

        self.command_pub = self.create_publisher(
            Twist, '/cmd_vel_wall_align', 10
        )
        self.status_pub = self.create_publisher(
            String, '/gemini/alignment/status', 10
        )
        self.preview_pub = self.create_publisher(
            CompressedImage, '/gemini/alignment/image_jpeg', sensor_qos()
        )
        self.create_subscription(
            Image, '/gemini/color/image_raw', self.color_callback, sensor_qos()
        )
        self.create_subscription(
            Image, '/gemini/depth/image_raw', self.depth_callback, sensor_qos()
        )
        self.create_subscription(
            BaseStatus, '/c50c/status', self.base_callback, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel_safe', self.safe_cmd_callback, 10
        )
        self.create_subscription(
            Bool, '/roscar/estop_state', self.estop_callback, latched_qos()
        )
        self.create_subscription(
            Bool, '/roscar/goal_active', self.goal_callback, latched_qos()
        )
        self.create_timer(0.1, self.watchdog)
        self.publish_status('启动中', '等待 Gemini 图像与深度')

    def base_callback(self, message):
        self.base_fault_bits = int(message.fault_bits)
        self.base_seen_at = time.monotonic()

    def safe_cmd_callback(self, message):
        self.safe_speed = math.hypot(
            float(message.linear.x), float(message.linear.y)
        )
        self.safe_cmd_seen_at = time.monotonic()

    def estop_callback(self, message):
        self.estop = bool(message.data)

    def goal_callback(self, message):
        self.goal_active = bool(message.data)

    def depth_callback(self, message):
        try:
            raw = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='passthrough'
            )
            self.depth_m = depth_to_meters(raw, message.encoding)
            self.last_depth_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(
                f'depth conversion failed: {exc}', throttle_duration_sec=5.0
            )

    def target_depth(self, target):
        if self.depth_m is None or time.monotonic() - self.last_depth_at > 0.5:
            self.get_logger().warning(
                'aligned depth frame is missing or stale',
                throttle_duration_sec=5.0,
            )
            return None
        height, width = self.depth_m.shape[:2]
        x, y, box_width, box_height = target['bbox']
        x0 = max(0, min(width, x + box_width // 5))
        x1 = max(0, min(width, x + box_width * 4 // 5))
        y0 = max(0, min(height, y + box_height // 5))
        y1 = max(0, min(height, y + box_height * 4 // 5))
        values = self.depth_m[y0:y1, x0:x1]
        values = values[np.isfinite(values) & (values > 0.10) & (values < 5.0)]
        if values.size < 100:
            positive = self.depth_m[
                np.isfinite(self.depth_m) & (self.depth_m > 0)
            ]
            frame_range = (
                f'{float(np.min(positive)):.3f}-'
                f'{float(np.max(positive)):.3f} m'
                if positive.size else 'no valid samples'
            )
            self.get_logger().warning(
                f'target depth ROI has {values.size} valid samples; '
                f'frame range {frame_range}',
                throttle_duration_sec=5.0,
            )
            return None
        return float(np.median(values))

    def safety_reason(self):
        now = time.monotonic()
        if self.estop:
            return '急停已触发'
        if self.goal_active:
            return '导航任务仍在执行'
        if self.base_fault_bits is None or now - self.base_seen_at > 0.5:
            return '底盘状态超时'
        if self.base_fault_bits:
            return f'底盘故障 0x{self.base_fault_bits:08X}'
        if not self.motion_started and (
            now - self.safe_cmd_seen_at > 0.5 or self.safe_speed > 0.01
        ):
            return '等待车体完全静止'
        return None

    def color_callback(self, message):
        self.last_image_at = time.monotonic()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            target = detect_red_target(image)
            annotated = image.copy()
            target_x = int(round(self.reference_center_x * image.shape[1]))
            target_y = int(round(self.reference_center_y * image.shape[0]))
            cv2.line(
                annotated, (target_x, 0), (target_x, image.shape[0] - 1),
                (0, 255, 255), 2,
            )
            cv2.line(
                annotated, (0, target_y), (image.shape[1] - 1, target_y),
                (0, 255, 255), 2,
            )
            if target is None:
                self.detections.clear()
                self.stop('未检测到唯一且可信的红砖目标')
            else:
                x, y, width, height = target['bbox']
                cv2.rectangle(
                    annotated, (x, y), (x + width, y + height), (0, 255, 0), 2
                )
                depth = self.target_depth(target)
                self.detections.append((
                    target['center_x'], target['center_y'],
                    target['area_ratio'], depth,
                ))
                self.process_detection(target, depth)
            self.publish_preview(message, annotated)
        except Exception as exc:
            self.stop(f'图像处理失败：{exc}')

    def process_detection(self, target, depth):
        if self.finished:
            return
        if len(self.detections) < self.detections.maxlen:
            self.stop('正在稳定曝光与目标检测')
            return
        centers_x = np.array([value[0] for value in self.detections])
        centers_y = np.array([value[1] for value in self.detections])
        areas = np.array([value[2] for value in self.detections])
        if (
            float(np.std(centers_x)) > 0.012
            or float(np.std(centers_y)) > 0.012
            or float(np.std(areas)) > 0.018
        ):
            self.stop('目标轮廓尚未稳定')
            return
        valid_depths = [
            value[3] for value in self.detections if value[3] is not None
        ]
        depth = (
            float(np.median(valid_depths)) if len(valid_depths) >= 4 else None
        )
        if depth is not None and depth < self.minimum_safe_depth_m:
            self.stop(f'目标过近，深度 {depth:.3f} m')
            return
        reason = self.safety_reason()
        if reason:
            self.stop(reason)
            return

        center_x = float(np.median(centers_x))
        center_y = float(np.median(centers_y))
        area = float(np.median(areas))
        vx, vy, aligned = visual_alignment_velocity(
            center_x,
            center_y,
            self.reference_center_x,
            self.reference_center_y,
            self.center_tolerance,
            self.vertical_tolerance,
            self.min_forward_speed,
            self.max_forward_speed,
            self.min_lateral_speed,
            self.max_lateral_speed,
        )
        if aligned:
            self.aligned_frames += 1
            self.reset_step()
            self.publish_zero()
            if self.aligned_frames >= 8:
                self.finished = True
                self.publish_status(
                    '已对齐',
                    '红砖中心已进入画面中央',
                    center=center_x,
                    center_y=center_y,
                    area=area,
                    depth=depth,
                )
            else:
                self.publish_status(
                    '确认中', '对齐结果连续确认', center=center_x,
                    center_y=center_y,
                    area=area, depth=depth,
                )
            return

        self.aligned_frames = 0
        if not self.motion_enabled:
            self.publish_zero()
            self.publish_status(
                '检测模式', '速度输出已禁用', center=center_x,
                center_y=center_y,
                area=area, depth=depth,
            )
            return
        now = time.monotonic()
        (vx, vy), self.move_until, self.settle_until = stepped_alignment_velocity(
            now,
            (vx, vy),
            self.step_command,
            self.move_until,
            self.settle_until,
            self.move_step_sec,
            self.settle_step_sec,
        )
        if vx == 0.0 and vy == 0.0:
            self.publish_zero()
            self.publish_status(
                '稳定中', '短步移动完成，等待车体与画面稳定',
                center=center_x, center_y=center_y, area=area, depth=depth,
            )
            return
        self.step_command = (vx, vy)
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        self.command_pub.publish(command)
        self.motion_started = True
        self.publish_status(
            '微调中',
            '低速匹配参考画面' if depth is not None else
            '深度受近距保护限制，按 RGB 参考画面低速匹配',
            center=center_x,
            center_y=center_y,
            area=area, depth=depth, vx=vx, vy=vy,
        )

    def stop(self, reason):
        self.aligned_frames = 0
        self.reset_step()
        self.publish_zero()
        self.publish_status('等待', reason)

    def reset_step(self):
        self.step_command = (0.0, 0.0)
        self.move_until = 0.0
        self.settle_until = 0.0

    def publish_zero(self):
        self.command_pub.publish(Twist())

    def publish_status(self, state, message, **values):
        payload = {'state': state, 'message': message, **values}
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(output)

    def publish_preview(self, source, image):
        preview = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(
            '.jpg', preview, [int(cv2.IMWRITE_JPEG_QUALITY), 72]
        )
        if not ok:
            return
        output = CompressedImage()
        output.header = source.header
        output.format = 'bgr8; jpeg compressed bgr8'
        output.data = encoded.tobytes()
        self.preview_pub.publish(output)

    def watchdog(self):
        elapsed = time.monotonic() - self.started_at
        if not self.finished and elapsed > self.maximum_runtime_sec:
            self.finished = True
            self.publish_zero()
            self.publish_status('超时', '视觉对齐超时，已停止车体')
        elif (
            self.last_image_at
            and time.monotonic() - self.last_image_at > 1.0
        ):
            self.stop('Gemini 图像超时')


def main(args=None):
    rclpy.init(args=args)
    node = GeminiAlignment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.publish_zero()
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
