import math

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .map_wall_filter import (
    OccupiedGridIndex,
    Transform2D,
    suppress_static_wall_endpoints,
)


class MapAwareGlobalScanFilter(Node):
    """Remove mapped-wall residuals from a global marking-only scan."""

    def __init__(self):
        super().__init__('map_aware_global_scan_filter')
        self.declare_parameter('input_topic', '/scan_slam_filtered')
        self.declare_parameter('output_topic', '/scan_global_obstacles')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('wall_proximity', 0.11)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('diagnostics_period_sec', 5.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.configured_map_frame = str(
            self.get_parameter('map_frame').value
        )
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )
        self.wall_proximity = max(
            0.0,
            float(self.get_parameter('wall_proximity').value),
        )
        self.tf_timeout = Duration(seconds=max(
            0.0,
            float(self.get_parameter('tf_timeout_sec').value),
        ))
        self.diagnostics_period_sec = max(
            1.0,
            float(self.get_parameter('diagnostics_period_sec').value),
        )

        self.occupied_grid = None
        self.map_frame = self.configured_map_frame
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        now_ns = self.get_clock().now().nanoseconds
        self.last_diagnostics_ns = now_ns
        self.stats = {
            'frames': 0,
            'filtered_frames': 0,
            'fail_open_no_map': 0,
            'fail_open_no_tf': 0,
            'finite_input': 0,
            'wall_suppressed': 0,
        }
        self.get_logger().info(
            f'Global marking scan filter {self.input_topic} -> '
            f'{self.output_topic}; map={self.map_topic}, '
            f'occupied>={self.occupied_threshold}, '
            f'wall_proximity={self.wall_proximity:.3f} m. '
            'Fail-open is mandatory; use the original scan as a separate '
            'clearing-only costmap source.'
        )

    @staticmethod
    def quaternion_yaw(quaternion):
        sine = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosine = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(sine, cosine)

    def map_callback(self, message):
        try:
            origin = message.info.origin
            self.occupied_grid = OccupiedGridIndex(
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=self.quaternion_yaw(origin.orientation),
                occupancy_data=message.data,
                occupied_threshold=self.occupied_threshold,
                proximity=self.wall_proximity,
            )
            self.map_frame = (
                message.header.frame_id or self.configured_map_frame
            )
            self.get_logger().info(
                f'Loaded static occupancy index: '
                f'{message.info.width}x{message.info.height} at '
                f'{message.info.resolution:.3f} m, '
                f'occupied_cells={len(self.occupied_grid.occupied)}, '
                f'frame={self.map_frame}'
            )
        except (TypeError, ValueError) as error:
            # Never retain an index whose geometry does not match the latest
            # map message.  Scan callbacks remain fail-open until a valid map.
            self.occupied_grid = None
            self.get_logger().error(f'Rejected invalid occupancy grid: {error}')

    @staticmethod
    def clone_scan(scan, ranges):
        output = LaserScan()
        output.header = scan.header
        output.angle_min = scan.angle_min
        output.angle_max = scan.angle_max
        output.angle_increment = scan.angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = scan.range_min
        output.range_max = scan.range_max
        output.ranges = ranges
        output.intensities = scan.intensities
        return output

    def publish_fail_open(self, scan, reason):
        # Dropping all data or failing closed could hide a real obstacle.  A
        # verbatim relay may duplicate a mapped wall temporarily, but that is
        # conservative and cannot make the global planner less collision-safe.
        self.stats[reason] += 1
        self.publisher.publish(self.clone_scan(scan, list(scan.ranges)))

    def scan_callback(self, scan):
        self.stats['frames'] += 1
        self.stats['finite_input'] += sum(
            1 for value in scan.ranges if math.isfinite(value)
        )
        grid = self.occupied_grid
        if grid is None:
            self.publish_fail_open(scan, 'fail_open_no_map')
            self.maybe_log_diagnostics()
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=self.tf_timeout,
            )
        except TransformException:
            self.publish_fail_open(scan, 'fail_open_no_tf')
            self.maybe_log_diagnostics()
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        filtered_ranges, suppressed = suppress_static_wall_endpoints(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            Transform2D(
                float(translation.x),
                float(translation.y),
                self.quaternion_yaw(rotation),
            ),
            grid,
        )
        self.stats['filtered_frames'] += 1
        self.stats['wall_suppressed'] += suppressed
        self.publisher.publish(self.clone_scan(scan, filtered_ranges))
        self.maybe_log_diagnostics()

    def maybe_log_diagnostics(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_diagnostics_ns) * 1e-9
        if elapsed < self.diagnostics_period_sec:
            return
        finite = self.stats['finite_input']
        ratio = (
            100.0 * self.stats['wall_suppressed'] / finite
            if finite
            else 0.0
        )
        self.get_logger().info(
            'global_scan_filter_diag '
            f'frames={self.stats["frames"]} '
            f'filtered={self.stats["filtered_frames"]} '
            f'fail_open_no_map={self.stats["fail_open_no_map"]} '
            f'fail_open_no_tf={self.stats["fail_open_no_tf"]} '
            f'wall_suppressed={self.stats["wall_suppressed"]} '
            f'suppression_ratio={ratio:.2f}%'
        )
        for key in self.stats:
            self.stats[key] = 0
        self.last_diagnostics_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = MapAwareGlobalScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
