"""Astra image previews and throttled local-obstacle point cloud."""

import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField

from .processing import (
    depth_to_meters,
    depth_to_palette_indices,
    filter_and_voxelize,
    xyz_from_pointcloud2,
)


def sensor_qos(depth=2):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class AstraDepthPipeline(Node):
    """Convert official Orbbec topics into bounded web and Nav2 feeds."""

    def __init__(self):
        super().__init__('astra_depth_pipeline')
        self.declare_parameter('depth_input_topic', '/astra/depth/image_raw')
        self.declare_parameter('points_input_topic', '/astra/depth/points')
        self.declare_parameter('depth_jpeg_topic', '/astra/depth/image_color_jpeg')
        self.declare_parameter('obstacle_points_topic', '/astra/depth/points_obstacles')
        self.declare_parameter('web_fps', 5.0)
        self.declare_parameter('pointcloud_fps', 10.0)
        self.declare_parameter('jpeg_quality', 68)
        self.declare_parameter('preview_width', 320)
        self.declare_parameter('preview_height', 240)
        self.declare_parameter('depth_display_near_m', 0.60)
        self.declare_parameter('depth_display_far_m', 4.00)
        self.declare_parameter('obstacle_min_range_m', 0.55)
        self.declare_parameter('obstacle_max_range_m', 2.80)
        self.declare_parameter('point_sample_stride', 4)
        self.declare_parameter('voxel_size_m', 0.03)

        self.depth_input_topic = str(self.get_parameter('depth_input_topic').value)
        self.points_input_topic = str(self.get_parameter('points_input_topic').value)
        self.depth_jpeg_topic = str(self.get_parameter('depth_jpeg_topic').value)
        self.obstacle_points_topic = str(
            self.get_parameter('obstacle_points_topic').value
        )
        self.web_interval = 1.0 / max(
            0.1, float(self.get_parameter('web_fps').value)
        )
        self.point_interval = 1.0 / max(
            0.1, float(self.get_parameter('pointcloud_fps').value)
        )
        self.jpeg_quality = min(
            100, max(30, int(self.get_parameter('jpeg_quality').value))
        )
        self.preview_width = max(
            160, int(self.get_parameter('preview_width').value)
        )
        self.preview_height = max(
            120, int(self.get_parameter('preview_height').value)
        )
        self.depth_near_m = float(
            self.get_parameter('depth_display_near_m').value
        )
        self.depth_far_m = float(
            self.get_parameter('depth_display_far_m').value
        )
        if not (
            math.isfinite(self.depth_near_m)
            and math.isfinite(self.depth_far_m)
            and self.depth_far_m > self.depth_near_m
        ):
            raise ValueError('invalid depth display range')
        self.obstacle_min_range_m = float(
            self.get_parameter('obstacle_min_range_m').value
        )
        self.obstacle_max_range_m = float(
            self.get_parameter('obstacle_max_range_m').value
        )
        self.point_sample_stride = max(
            1, int(self.get_parameter('point_sample_stride').value)
        )
        self.voxel_size_m = max(
            0.0, float(self.get_parameter('voxel_size_m').value)
        )

        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()
        self.depth_pub = self.create_publisher(
            CompressedImage, self.depth_jpeg_topic, sensor_qos(1)
        )
        self.points_pub = self.create_publisher(
            PointCloud2, self.obstacle_points_topic, sensor_qos(2)
        )
        self.create_subscription(
            Image,
            self.depth_input_topic,
            self.depth_callback,
            sensor_qos(1),
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PointCloud2,
            self.points_input_topic,
            self.points_callback,
            sensor_qos(1),
            callback_group=self.callback_group,
        )

        self._throttle_lock = threading.Lock()
        self._last_depth_process = 0.0
        self._last_points_process = 0.0
        self._counts = {'depth': 0, 'points': 0}
        self._last_report_counts = dict(self._counts)
        self._last_report_time = time.monotonic()
        self.create_timer(10.0, self.report_health)
        self.get_logger().info(
            'Astra pipeline ready: fixed 256-level Turbo depth '
            f'{self.depth_near_m:.2f}-{self.depth_far_m:.2f} m; '
            f'local obstacle cloud {self.obstacle_min_range_m:.2f}-'
            f'{self.obstacle_max_range_m:.2f} m'
        )

    def _claim_interval(self, attribute, interval):
        now = time.monotonic()
        with self._throttle_lock:
            last = getattr(self, attribute)
            if now - last < interval:
                return False
            setattr(self, attribute, now)
        return True

    def _publish_jpeg(
        self,
        publisher,
        header,
        bgr_image,
        interpolation=cv2.INTER_AREA,
    ):
        if (
            bgr_image.shape[1] != self.preview_width
            or bgr_image.shape[0] != self.preview_height
        ):
            bgr_image = cv2.resize(
                bgr_image,
                (self.preview_width, self.preview_height),
                interpolation=interpolation,
            )
        ok, encoded = cv2.imencode(
            '.jpg',
            bgr_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError('OpenCV JPEG encoding failed')
        output = CompressedImage()
        output.header = header
        output.format = 'bgr8; jpeg compressed bgr8'
        output.data = encoded.tobytes()
        publisher.publish(output)

    def depth_callback(self, message):
        if not self._claim_interval('_last_depth_process', self.web_interval):
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            depth_m = depth_to_meters(raw, message.encoding)
            indices, valid = depth_to_palette_indices(
                depth_m, self.depth_near_m, self.depth_far_m
            )
            color = cv2.applyColorMap(indices, cv2.COLORMAP_TURBO)
            color[~valid] = 0
            self._publish_jpeg(
                self.depth_pub,
                message.header,
                color,
                interpolation=cv2.INTER_NEAREST,
            )
            self._counts['depth'] += 1
        except Exception as exc:
            self.get_logger().error(
                f'depth preview conversion failed: {exc}',
                throttle_duration_sec=5.0,
            )

    def points_callback(self, message):
        if not self._claim_interval('_last_points_process', self.point_interval):
            return
        try:
            points = xyz_from_pointcloud2(message, self.point_sample_stride)
            points = filter_and_voxelize(
                points,
                self.obstacle_min_range_m,
                self.obstacle_max_range_m,
                self.voxel_size_m,
            )
            output = PointCloud2()
            output.header = message.header
            output.height = 1
            output.width = int(points.shape[0])
            output.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            output.is_bigendian = False
            output.point_step = 12
            output.row_step = output.width * output.point_step
            output.data = np.asarray(points, dtype='<f4').tobytes(order='C')
            output.is_dense = True
            self.points_pub.publish(output)
            self._counts['points'] += 1
        except Exception as exc:
            self.get_logger().error(
                f'point cloud processing failed: {exc}',
                throttle_duration_sec=5.0,
            )

    def report_health(self):
        now = time.monotonic()
        elapsed = max(0.001, now - self._last_report_time)
        rates = {
            name: (self._counts[name] - self._last_report_counts[name]) / elapsed
            for name in self._counts
        }
        self._last_report_counts = dict(self._counts)
        self._last_report_time = now
        self.get_logger().info(
            'Astra output rates: '
            f'depth {rates["depth"]:.1f} Hz, '
            f'obstacle cloud {rates["points"]:.1f} Hz'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AstraDepthPipeline()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
