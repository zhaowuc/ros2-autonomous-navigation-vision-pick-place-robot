import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - reported clearly at runtime on robot
    serial = None
    SerialException = Exception


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
CONTROL_FRAME_SIZE = 11
STATE_FRAME_SIZE = 24


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def latest_command_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def int16_to_bytes_scaled(value):
    scaled = int(round(value * 1000.0))
    scaled = int(clamp(scaled, -32768, 32767))
    return struct.pack('>h', scaled)


def read_i16_be(data, index):
    return struct.unpack('>h', bytes(data[index:index + 2]))[0]


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def normalized_odom_source(value):
    text = str(value).strip().lower()
    if text in ('command', 'cmd', 'cmd_vel'):
        return 'commanded'
    if text in ('feedback', 'serial', 'base', 'mcu'):
        return 'feedback'
    return 'feedback'


def motion_mode(vx, vy, wz, deadband=1e-4):
    has_vx = abs(vx) > deadband
    has_vy = abs(vy) > deadband
    has_wz = abs(wz) > deadband
    linear = has_vx or has_vy
    angular = has_wz
    if not linear and not angular:
        return 'zero'
    if angular and not linear:
        return 'wz_pos' if wz > 0.0 else 'wz_neg'
    if linear and not angular:
        if has_vx and not has_vy:
            return 'vx_pos' if vx > 0.0 else 'vx_neg'
        if has_vy and not has_vx:
            return 'vy_pos' if vy > 0.0 else 'vy_neg'
    return 'mixed'


def bcc(data):
    value = 0
    for item in data:
        value ^= item
    return value & 0xFF


class C50CBaseDriver(Node):
    def __init__(self):
        super().__init__('c50c_base_driver')

        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31014980-if00',
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.02)
        self.declare_parameter('max_vx', 0.08)
        self.declare_parameter('max_vy', 0.08)
        self.declare_parameter('max_wz', 0.20)
        self.declare_parameter('cmd_timeout', 0.12)
        self.declare_parameter('mode_switch_stop_seconds', 0.0)
        self.declare_parameter('boot_hold_seconds', 0.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('odom_source', 'feedback')
        self.declare_parameter('odom_vx_scale', 1.0)
        self.declare_parameter('odom_vy_scale', 1.0)
        self.declare_parameter('odom_wz_scale', 1.0)
        self.declare_parameter('odom_wz_pos_scale', 1.0)
        self.declare_parameter('odom_wz_neg_scale', 1.0)
        self.declare_parameter('odom_linear_deadband', 0.003)
        self.declare_parameter('odom_angular_deadband', 0.003)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('publish_rate', 100.0)

        self.port = self.get_parameter('port').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.mode_switch_stop_seconds = float(self.get_parameter('mode_switch_stop_seconds').value)
        self.boot_hold_seconds = float(self.get_parameter('boot_hold_seconds').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.odom_source = normalized_odom_source(self.get_parameter('odom_source').value)
        self.odom_vx_scale = float(self.get_parameter('odom_vx_scale').value)
        self.odom_vy_scale = float(self.get_parameter('odom_vy_scale').value)
        self.odom_wz_scale = float(self.get_parameter('odom_wz_scale').value)
        self.odom_wz_pos_scale = float(self.get_parameter('odom_wz_pos_scale').value)
        self.odom_wz_neg_scale = float(self.get_parameter('odom_wz_neg_scale').value)
        self.odom_linear_deadband = float(self.get_parameter('odom_linear_deadband').value)
        self.odom_angular_deadband = float(self.get_parameter('odom_angular_deadband').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.imu_topic = self.get_parameter('imu_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, latest_command_qos())
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.status_pub = self.create_publisher(String, '/c50c/status', 10)
        self.reset_odom_srv = self.create_service(Empty, '/reset_odom', self.reset_odom_callback)
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if self.publish_odom_tf
            else None
        )

        self.lock = threading.Lock()
        self.serial_port = None
        self.rx_buffer = bytearray()
        self.connected = False
        self.last_status_text = ''
        self.last_dry_run_frame_hex = ''
        self.last_dry_run_log_time = 0.0

        self.start_time = time.monotonic()
        self.last_cmd_time = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_motion_mode = 'zero'
        self.mode_switch_zero_until = 0.0
        self.zero_settle_until = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = None

        self.timer = self.create_timer(1.0 / publish_rate, self.control_timer)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'C50C driver ready: port={self.port}, baudrate={self.baudrate}, '
            f'cmd_topic={self.cmd_vel_topic}, '
            f'max=({self.max_vx:.2f}, {self.max_vy:.2f}, {self.max_wz:.2f}), '
            f'odom_source={self.odom_source}, boot_hold={self.boot_hold_seconds:.1f}s, '
            f'dry_run={self.dry_run}'
        )

    def cmd_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
            vy = clamp(msg.linear.y, -self.max_vy, self.max_vy)
            wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
            new_mode = motion_mode(vx, vy, wz)
            old_mode = self.last_motion_mode
            if new_mode == 'zero':
                self.mode_switch_zero_until = 0.0
            elif old_mode not in ('zero', new_mode):
                if self.mode_switch_stop_seconds > 0.0:
                    self.mode_switch_zero_until = max(
                        self.mode_switch_zero_until,
                        now + self.mode_switch_stop_seconds,
                    )
            elif self.mode_switch_zero_until > now:
                hold_zero = True
            else:
                self.mode_switch_zero_until = 0.0
            self.zero_settle_until = 0.0
            self.last_motion_mode = new_mode
            self.target_vx = vx
            self.target_vy = vy
            self.target_wz = wz
            self.last_cmd_time = now
        # The 100 Hz control timer is the sole serial writer.  Publishing
        # here as well allowed a callback and the timer to reorder old/new
        # frames around a stop command.

    def reset_odom_callback(self, _request, response):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.last_odom_time = None
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
            self.last_cmd_time = 0.0
            self.last_motion_mode = 'zero'
            self.mode_switch_zero_until = 0.0
            self.zero_settle_until = 0.0
        self.get_logger().info('odometry reset to x=0, y=0, yaw=0')
        return response

    def connect_serial(self):
        if self.dry_run:
            self.set_status('dry_run: serial disabled')
            return False
        if serial is None:
            self.set_status('error: python3-serial not installed')
            return False
        if self.serial_port and self.serial_port.is_open:
            return True
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=0.05,
            )
            self.connected = True
            self.set_status(f'connected: {self.port}')
            return True
        except SerialException as exc:
            self.connected = False
            self.serial_port = None
            self.set_status(f'disconnected: {exc}')
            return False

    def set_status(self, text):
        if text != self.last_status_text:
            if text.startswith('connected'):
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            self.last_status_text = text

    def publish_status(self):
        msg = String()
        elapsed = time.monotonic() - self.start_time
        if elapsed < self.boot_hold_seconds:
            phase = f'boot_hold {self.boot_hold_seconds - elapsed:.1f}s'
        else:
            phase = 'ready'
        msg.data = f'{self.last_status_text or "unknown"}; {phase}'
        self.status_pub.publish(msg)

    def flush_serial_output(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            self.serial_port.reset_output_buffer()
        except SerialException:
            pass

    def control_timer(self):
        if not self.dry_run:
            self.connect_serial()
            self.read_available_frames()

        now = time.monotonic()
        with self.lock:
            cmd_age = now - self.last_cmd_time if self.last_cmd_time > 0 else float('inf')
            if (
                now - self.start_time < self.boot_hold_seconds
                or cmd_age > self.cmd_timeout
                or now < self.mode_switch_zero_until
            ):
                vx, vy, wz = 0.0, 0.0, 0.0
            else:
                vx, vy, wz = self.target_vx, self.target_vy, self.target_wz

        self.write_velocity(vx, vy, wz)
        if self.odom_source == 'commanded':
            if self.connected or self.dry_run:
                self.publish_odometry_from_motion(vx, vy, wz)
            else:
                self.publish_odometry_from_motion(0.0, 0.0, 0.0)

    def write_velocity(self, vx, vy, wz):
        frame = bytearray([FRAME_HEADER, 0x00, 0x00])
        frame.extend(int16_to_bytes_scaled(vx))
        frame.extend(int16_to_bytes_scaled(vy))
        frame.extend(int16_to_bytes_scaled(wz))
        frame.append(bcc(frame))
        frame.append(FRAME_TAIL)

        if len(frame) != CONTROL_FRAME_SIZE:
            self.get_logger().error('internal error: invalid C50C control frame length')
            return

        frame_hex = frame.hex(' ')
        if self.dry_run:
            now = time.monotonic()
            if frame_hex != self.last_dry_run_frame_hex or now - self.last_dry_run_log_time > 1.0:
                self.get_logger().info(
                    f'dry_run tx vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}: {frame_hex}'
                )
                self.last_dry_run_frame_hex = frame_hex
                self.last_dry_run_log_time = now
            return

        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            self.serial_port.write(frame)
        except SerialException as exc:
            self.connected = False
            self.set_status(f'write error: {exc}')
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    def read_available_frames(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return
            data = self.serial_port.read(waiting)
        except SerialException as exc:
            self.connected = False
            self.set_status(f'read error: {exc}')
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            return
        if not data:
            return
        self.rx_buffer.extend(data)

        while len(self.rx_buffer) >= STATE_FRAME_SIZE:
            header_index = self.rx_buffer.find(bytes([FRAME_HEADER]))
            if header_index < 0:
                self.rx_buffer.clear()
                return
            if header_index > 0:
                del self.rx_buffer[:header_index]
            if len(self.rx_buffer) < STATE_FRAME_SIZE:
                return
            candidate = self.rx_buffer[:STATE_FRAME_SIZE]
            if candidate[-1] != FRAME_TAIL:
                del self.rx_buffer[0]
                continue
            if candidate[22] != bcc(candidate[:22]):
                del self.rx_buffer[0]
                continue
            del self.rx_buffer[:STATE_FRAME_SIZE]
            self.handle_state_frame(candidate)

    def handle_state_frame(self, frame):
        stamp = self.get_clock().now()
        feedback_vx = read_i16_be(frame, 2) / 1000.0
        feedback_vy = read_i16_be(frame, 4) / 1000.0
        feedback_wz = read_i16_be(frame, 6) / 1000.0
        ax = read_i16_be(frame, 8) / 1000.0
        ay = read_i16_be(frame, 10) / 1000.0
        az = read_i16_be(frame, 12) / 1000.0
        gx = read_i16_be(frame, 14) / 1000.0
        gy = read_i16_be(frame, 16) / 1000.0
        gz = read_i16_be(frame, 18) / 1000.0
        voltage = read_i16_be(frame, 20) / 1000.0

        if self.odom_source == 'feedback':
            self.publish_odometry_from_motion(feedback_vx, feedback_vy, feedback_wz, stamp)

        self.publish_imu_battery(stamp, ax, ay, az, gx, gy, gz, voltage)

    def publish_odometry_from_motion(self, vx, vy, wz, stamp=None):
        stamp = stamp or self.get_clock().now()
        vx *= self.odom_vx_scale
        vy *= self.odom_vy_scale
        if wz > 0.0:
            wz *= self.odom_wz_pos_scale
        elif wz < 0.0:
            wz *= self.odom_wz_neg_scale
        else:
            wz *= self.odom_wz_scale
        if abs(vx) < self.odom_linear_deadband:
            vx = 0.0
        if abs(vy) < self.odom_linear_deadband:
            vy = 0.0
        if abs(wz) < self.odom_angular_deadband:
            wz = 0.0

        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = (stamp - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = stamp

        if 0.0 < dt < 1.0:
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            self.x += (vx * cos_yaw - vy * sin_yaw) * dt
            self.y += (vx * sin_yaw + vy * cos_yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        quat = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.04
        odom.pose.covariance[7] = 0.04
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.04
        odom.twist.covariance[7] = 0.04
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_imu_battery(self, stamp, ax, ay, az, gx, gy, gz, voltage):
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.imu_frame_id
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        self.battery_pub.publish(Float32(data=float(voltage)))

    def stop_robot(self):
        for _ in range(5):
            self.write_velocity(0.0, 0.0, 0.0)
            time.sleep(0.02)

    def destroy_node(self):
        self.stop_robot()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = C50CBaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if 'context is not valid' not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
