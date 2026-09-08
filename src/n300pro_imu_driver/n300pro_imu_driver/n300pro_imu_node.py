"""ROS 2 node for an N300Pro IMU using the HiPNUC HI91 protocol."""

import math
import threading
import time
from typing import Iterable, List

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Temperature
import serial

from .orientation import quaternion_wxyz_to_rpy
from .protocol import Hi91Data, HiPNUCDecoder


STANDARD_GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0
MICROTESLA_TO_TESLA = 1.0e-6


def diagonal_covariance(stddev: float) -> List[float]:
    variance = stddev * stddev
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


class N300ProImuNode(Node):
    """Read, validate, convert and publish N300Pro measurements."""

    def __init__(self) -> None:
        super().__init__("n300pro_imu")

        self.declare_parameter(
            "serial_port",
            "/dev/n300_imu",
        )
        self.declare_parameter("baud_rate", 115200)
        # Keep the N300Pro namespace distinct from the C50C legacy IMU.
        self.declare_parameter("frame_id", "n300_imu_link")
        self.declare_parameter("imu_topic", "/imu/n300/data")
        self.declare_parameter("mag_topic", "/imu/n300/mag")
        self.declare_parameter("temperature_topic", "/imu/n300/temperature")
        self.declare_parameter("pressure_topic", "/imu/n300/pressure")
        self.declare_parameter("rpy_topic", "/imu/n300/rpy")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("expected_rate_hz", 100.0)
        self.declare_parameter("minimum_rate_hz", 90.0)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("serial_reopen_timeout_sec", 2.0)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("orientation_stddev", 0.035)
        self.declare_parameter("angular_velocity_stddev", 0.003)
        self.declare_parameter("linear_acceleration_stddev", 0.05)
        self.declare_parameter("magnetic_field_stddev", 5.0e-6)
        self.declare_parameter("temperature_stddev", 1.0)
        self.declare_parameter("pressure_stddev", 2.0)

        self._port = self.get_parameter("serial_port").value
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._imu_topic = self.get_parameter("imu_topic").value
        self._mag_topic = self.get_parameter("mag_topic").value
        self._temperature_topic = self.get_parameter("temperature_topic").value
        self._pressure_topic = self.get_parameter("pressure_topic").value
        self._rpy_topic = self.get_parameter("rpy_topic").value
        self._diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self._expected_rate = float(self.get_parameter("expected_rate_hz").value)
        self._minimum_rate = float(self.get_parameter("minimum_rate_hz").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self._serial_reopen_timeout = float(
            self.get_parameter("serial_reopen_timeout_sec").value
        )
        self._reconnect_delay = float(self.get_parameter("reconnect_delay_sec").value)

        orientation_stddev = float(self.get_parameter("orientation_stddev").value)
        angular_stddev = float(self.get_parameter("angular_velocity_stddev").value)
        acceleration_stddev = float(self.get_parameter("linear_acceleration_stddev").value)
        magnetic_stddev = float(self.get_parameter("magnetic_field_stddev").value)
        temperature_stddev = float(self.get_parameter("temperature_stddev").value)
        pressure_stddev = float(self.get_parameter("pressure_stddev").value)
        if self._baud_rate <= 0:
            raise ValueError("baud_rate must be positive")
        if not self._port:
            raise ValueError("serial_port must not be empty")
        if not self._frame_id:
            raise ValueError("frame_id must not be empty")
        topic_parameters = {
            "imu_topic": self._imu_topic,
            "mag_topic": self._mag_topic,
            "temperature_topic": self._temperature_topic,
            "pressure_topic": self._pressure_topic,
            "rpy_topic": self._rpy_topic,
            "diagnostics_topic": self._diagnostics_topic,
        }
        empty_topics = [name for name, value in topic_parameters.items() if not value]
        if empty_topics:
            raise ValueError(f"topic parameters must not be empty: {empty_topics}")
        if self._expected_rate <= 0.0 or self._minimum_rate < 0.0:
            raise ValueError(
                "expected_rate_hz must be positive and minimum_rate_hz nonnegative"
            )
        if self._stale_timeout <= 0.0 or self._serial_reopen_timeout <= 0.0:
            raise ValueError("stale and serial reopen timeouts must be positive")
        if self._reconnect_delay < 0.0:
            raise ValueError("reconnect_delay_sec must be nonnegative")
        if min(
            orientation_stddev,
            angular_stddev,
            acceleration_stddev,
            magnetic_stddev,
            temperature_stddev,
            pressure_stddev,
        ) < 0.0:
            raise ValueError("measurement standard deviations must be nonnegative")
        self._orientation_covariance = diagonal_covariance(orientation_stddev)
        self._angular_covariance = diagonal_covariance(angular_stddev)
        self._acceleration_covariance = diagonal_covariance(acceleration_stddev)
        self._magnetic_covariance = diagonal_covariance(magnetic_stddev)
        self._temperature_variance = temperature_stddev * temperature_stddev
        self._pressure_variance = pressure_stddev * pressure_stddev

        self._imu_pub = self.create_publisher(
            Imu, self._imu_topic, qos_profile_sensor_data
        )
        self._mag_pub = self.create_publisher(
            MagneticField,
            self._mag_topic,
            qos_profile_sensor_data,
        )
        self._temperature_pub = self.create_publisher(
            Temperature,
            self._temperature_topic,
            qos_profile_sensor_data,
        )
        self._pressure_pub = self.create_publisher(
            FluidPressure,
            self._pressure_topic,
            qos_profile_sensor_data,
        )
        self._rpy_pub = self.create_publisher(
            Vector3Stamped,
            self._rpy_topic,
            qos_profile_sensor_data,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )

        self._decoder = HiPNUCDecoder()
        self._decoder_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._serial = None
        self._connected = False
        self._last_error = "not connected yet"
        self._frame_count = 0
        self._invalid_samples = 0
        self._serial_errors = 0
        self._device_timestamp_jumps = 0
        self._last_device_time_ms = None
        self._last_frame_monotonic = None
        self._last_main_status = 0
        self._last_raw_euler_deg = None

        self._last_diag_monotonic = time.monotonic()
        self._last_diag_frames = 0
        self._last_diag_crc_errors = 0
        self._last_diag_timestamp_jumps = 0
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="n300pro_serial_reader",
            daemon=True,
        )
        self._reader_thread.start()
        self.get_logger().info(
            f"N300Pro driver configured for {self._port} at {self._baud_rate} bps; "
            f"publishing {self._imu_topic} in frame {self._frame_id}"
        )

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                device = serial.Serial(
                    port=self._port,
                    baudrate=self._baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2,
                    write_timeout=0.2,
                    exclusive=True,
                )
                device.reset_input_buffer()
                with self._decoder_lock:
                    self._decoder.reset_buffer()
                with self._state_lock:
                    self._serial = device
                    self._connected = True
                    self._last_error = ""
                    self._last_device_time_ms = None
                self.get_logger().info(f"Opened N300Pro serial port {self._port}")
                last_valid_frame = time.monotonic()

                while not self._stop_event.is_set():
                    chunk = device.read(82)
                    with self._decoder_lock:
                        samples = self._decoder.feed(chunk) if chunk else []
                    if samples:
                        last_valid_frame = time.monotonic()
                    elif time.monotonic() - last_valid_frame > self._serial_reopen_timeout:
                        raise serial.SerialException(
                            "no valid HI91 frame received for "
                            f"{self._serial_reopen_timeout:.1f} seconds"
                        )
                    for sample in samples:
                        self._publish_sample(sample)
            except (OSError, serial.SerialException) as exc:
                if not self._stop_event.is_set():
                    self.get_logger().error(f"N300Pro serial error: {exc}")
                with self._state_lock:
                    self._serial_errors += 1
                    self._last_error = str(exc)
            except Exception as exc:  # Keep the reconnect loop alive on bad input.
                if not self._stop_event.is_set():
                    self.get_logger().error(f"N300Pro reader error: {exc}")
                with self._state_lock:
                    self._serial_errors += 1
                    self._last_error = repr(exc)
            finally:
                with self._state_lock:
                    device = self._serial
                    self._serial = None
                    self._connected = False
                if device is not None and device.is_open:
                    try:
                        device.close()
                    except serial.SerialException:
                        pass

            self._stop_event.wait(self._reconnect_delay)

    def _publish_sample(self, sample: Hi91Data) -> None:
        scalar_values = (
            sample.temperature_c,
            sample.pressure_pa,
            *sample.acceleration_g,
            *sample.angular_velocity_dps,
            *sample.magnetic_field_ut,
            *sample.euler_deg,
            *sample.quaternion_wxyz,
        )
        quaternion_norm = math.sqrt(
            sum(value * value for value in sample.quaternion_wxyz)
        )
        if not finite(scalar_values) or not 0.5 < quaternion_norm < 1.5:
            with self._state_lock:
                self._invalid_samples += 1
            return

        quaternion = tuple(value / quaternion_norm for value in sample.quaternion_wxyz)
        roll, pitch, yaw = quaternion_wxyz_to_rpy(quaternion)
        stamp = self.get_clock().now().to_msg()

        # Do not hide installation mistakes with per-field sign changes here. HI91
        # vectors and attitude are published in the declared sensor frame. The
        # device output convention and physical mounting must be verified as ROS
        # ENU/FLU before this data is enabled in a state estimator.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._frame_id
        imu.orientation.w = quaternion[0]
        imu.orientation.x = quaternion[1]
        imu.orientation.y = quaternion[2]
        imu.orientation.z = quaternion[3]
        imu.orientation_covariance = self._orientation_covariance
        imu.angular_velocity.x = sample.angular_velocity_dps[0] * DEG_TO_RAD
        imu.angular_velocity.y = sample.angular_velocity_dps[1] * DEG_TO_RAD
        imu.angular_velocity.z = sample.angular_velocity_dps[2] * DEG_TO_RAD
        imu.angular_velocity_covariance = self._angular_covariance
        imu.linear_acceleration.x = sample.acceleration_g[0] * STANDARD_GRAVITY
        imu.linear_acceleration.y = sample.acceleration_g[1] * STANDARD_GRAVITY
        imu.linear_acceleration.z = sample.acceleration_g[2] * STANDARD_GRAVITY
        imu.linear_acceleration_covariance = self._acceleration_covariance
        self._imu_pub.publish(imu)

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = self._frame_id
        magnetic.magnetic_field.x = sample.magnetic_field_ut[0] * MICROTESLA_TO_TESLA
        magnetic.magnetic_field.y = sample.magnetic_field_ut[1] * MICROTESLA_TO_TESLA
        magnetic.magnetic_field.z = sample.magnetic_field_ut[2] * MICROTESLA_TO_TESLA
        magnetic.magnetic_field_covariance = self._magnetic_covariance
        self._mag_pub.publish(magnetic)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.header.frame_id = self._frame_id
        temperature.temperature = sample.temperature_c
        temperature.variance = self._temperature_variance
        self._temperature_pub.publish(temperature)

        pressure = FluidPressure()
        pressure.header.stamp = stamp
        pressure.header.frame_id = self._frame_id
        pressure.fluid_pressure = sample.pressure_pa
        pressure.variance = self._pressure_variance
        self._pressure_pub.publish(pressure)

        rpy = Vector3Stamped()
        rpy.header.stamp = stamp
        rpy.header.frame_id = self._frame_id
        # The HI91 native Euler payload does not use ROS roll,pitch ordering.
        # Derive the public ROS RPY topic from the same normalized quaternion
        # published in sensor_msgs/Imu so both attitude views stay consistent.
        rpy.vector.x = roll
        rpy.vector.y = pitch
        rpy.vector.z = yaw
        self._rpy_pub.publish(rpy)

        now = time.monotonic()
        with self._state_lock:
            if self._last_device_time_ms is not None:
                delta_ms = (
                    sample.device_time_ms - self._last_device_time_ms
                ) & 0xFFFFFFFF
                expected_period_ms = 1000.0 / self._expected_rate
                jump_threshold_ms = max(100.0, expected_period_ms * 5.0)
                if delta_ms == 0 or delta_ms > jump_threshold_ms:
                    self._device_timestamp_jumps += 1
            self._last_device_time_ms = sample.device_time_ms
            self._last_frame_monotonic = now
            self._last_main_status = sample.main_status
            self._last_raw_euler_deg = sample.euler_deg
            self._frame_count += 1

    @staticmethod
    def _status_flags(status: int) -> List[str]:
        names = {
            3: "WB_CONV",
            4: "MAG_DIST",
            5: "ACC_SAT",
            6: "GYR_SAT",
            7: "ATT_CONV",
            10: "MAG_AIDING",
            11: "UTC_UNSYNCED",
            12: "SOUT_PULSE",
        }
        return [name for bit, name in names.items() if status & (1 << bit)]

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            connected = self._connected
            last_error = self._last_error
            frames = self._frame_count
            invalid_samples = self._invalid_samples
            serial_errors = self._serial_errors
            timestamp_jumps = self._device_timestamp_jumps
            last_frame = self._last_frame_monotonic
            main_status = self._last_main_status
            raw_euler_deg = self._last_raw_euler_deg

        elapsed = max(now - self._last_diag_monotonic, 1.0e-6)
        rate = (frames - self._last_diag_frames) / elapsed
        with self._decoder_lock:
            decoder_crc_errors = self._decoder.crc_errors
            decoder_invalid_lengths = self._decoder.invalid_lengths
            decoder_invalid_payloads = self._decoder.invalid_payloads
            decoder_discarded_bytes = self._decoder.discarded_bytes
        new_crc_errors = decoder_crc_errors - self._last_diag_crc_errors
        new_timestamp_jumps = timestamp_jumps - self._last_diag_timestamp_jumps
        age = math.inf if last_frame is None else now - last_frame
        alarm_mask = main_status & 0x00F8

        if not connected:
            level = DiagnosticStatus.ERROR
            message = "serial port disconnected"
        elif age > self._stale_timeout:
            level = DiagnosticStatus.ERROR
            message = "IMU data stale"
        elif rate < self._minimum_rate:
            level = DiagnosticStatus.WARN
            message = f"low sample rate: {rate:.1f} Hz"
        elif alarm_mask:
            level = DiagnosticStatus.WARN
            message = "device status alarm"
        elif new_crc_errors or new_timestamp_jumps:
            level = DiagnosticStatus.WARN
            message = "stream anomaly observed"
        else:
            level = DiagnosticStatus.OK
            message = "N300Pro stream healthy"

        values = {
            "serial_port": self._port,
            "baud_rate": str(self._baud_rate),
            "frame_id": self._frame_id,
            "imu_topic": self._imu_topic,
            "rpy_topic": self._rpy_topic,
            "connected": str(connected).lower(),
            "sample_rate_hz": f"{rate:.3f}",
            "expected_rate_hz": f"{self._expected_rate:.3f}",
            "last_sample_age_sec": "inf" if math.isinf(age) else f"{age:.6f}",
            "frames_ok": str(frames),
            "crc_errors": str(decoder_crc_errors),
            "invalid_lengths": str(decoder_invalid_lengths),
            "invalid_payloads": str(decoder_invalid_payloads),
            "invalid_samples": str(invalid_samples),
            "discarded_bytes": str(decoder_discarded_bytes),
            "serial_errors": str(serial_errors),
            "device_timestamp_jumps": str(timestamp_jumps),
            "main_status": f"0x{main_status:04x}",
            "status_flags": ",".join(self._status_flags(main_status)) or "none",
            "last_error": last_error or "none",
            "timestamp_source": "host ROS clock",
            "axis_mapping": "native HI91; no software sign or axis remap",
            "rpy_source": "normalized quaternion; ROS roll,pitch,yaw",
            "raw_euler_native_deg": (
                "unavailable"
                if raw_euler_deg is None
                else ",".join(f"{value:.6f}" for value in raw_euler_deg)
            ),
            "raw_euler_note": "device-native diagnostic only; not published as ROS RPY",
        }

        status = DiagnosticStatus()
        status.level = level
        status.name = f"{self.get_name()}: N300Pro HI91"
        status.message = message
        status.hardware_id = self._port
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)

        self._last_diag_monotonic = now
        self._last_diag_frames = frames
        self._last_diag_crc_errors = decoder_crc_errors
        self._last_diag_timestamp_jumps = timestamp_jumps

    def destroy_node(self) -> bool:
        self._stop_event.set()
        with self._state_lock:
            device = self._serial
        if device is not None and device.is_open:
            try:
                device.close()
            except serial.SerialException:
                pass
        if self._reader_thread.is_alive():
            try:
                self._reader_thread.join(timeout=2.0)
            except KeyboardInterrupt:
                # A launch supervisor can forward a second SIGINT while the
                # reader thread is already being stopped.
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = N300ProImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
