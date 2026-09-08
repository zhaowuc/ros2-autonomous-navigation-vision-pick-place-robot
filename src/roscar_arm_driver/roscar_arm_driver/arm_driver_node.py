from functools import partial
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import serial
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .protocol import (
    ACTION_COMMANDS,
    command_for,
    pose_command,
    stored_group_command,
    validate_sequence,
)


class ArmDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("roscar_arm")
        self.declare_parameter("serial_port", "/dev/roscar_arm")
        self.declare_parameter("baudrate", 115200)
        self._port_name = str(self.get_parameter("serial_port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._port = None
        self._lock = threading.RLock()
        self._motion_generation = 0
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(String, "~/command", self._command_callback, 10)

        for action in ACTION_COMMANDS:
            self.create_service(
                Trigger,
                f"~/{action}",
                partial(self._run_action, action),
            )
        self.create_service(Trigger, "~/probe", self._probe)
        self.get_logger().info(
            f"Zhongling arm ready on {self._port_name} at {self._baudrate} baud"
        )

    def _connect(self):
        if self._port is not None and self._port.is_open:
            return self._port
        self._port = serial.Serial(
            self._port_name,
            self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1.0,
            exclusive=True,
        )
        return self._port

    def _probe(self, _request, response):
        with self._lock:
            try:
                self._connect()
                response.success = True
                response.message = f"opened {self._port_name}; no motion command sent"
            except (OSError, serial.SerialException) as exc:
                self._close()
                response.success = False
                response.message = str(exc)
        return response

    def _write(self, command: bytes) -> None:
        with self._lock:
            port = self._connect()
            written = port.write(command)
            port.flush()
            if written != len(command):
                raise IOError(f"short serial write: {written}/{len(command)}")

    def _new_motion(self) -> int:
        with self._lock:
            self._motion_generation += 1
            return self._motion_generation

    def _publish_status(self, success: bool, message: str) -> None:
        status = String()
        status.data = json.dumps(
            {"success": bool(success), "message": str(message)},
            ensure_ascii=False,
        )
        self._status_pub.publish(status)

    def _command_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("arm command must be a JSON object")
            kind = payload.get("type")
            generation = self._new_motion()
            if kind == "stop":
                self._write(command_for("stop"))
                self._publish_status(True, "机械臂已发送停止指令")
            elif kind == "stored":
                command = stored_group_command(
                    payload.get("start"), payload.get("end"), payload.get("repeat", 1)
                )
                self._write(command)
                self._publish_status(True, f"已执行板内动作 {command.decode('ascii')}")
            elif kind == "pose":
                command = pose_command(payload.get("pulses", ()), payload.get("duration_ms"))
                self._write(command)
                self._publish_status(True, "已发送 6 舵机姿态")
            elif kind == "sequence":
                frames = validate_sequence(payload.get("frames", ()))
                threading.Thread(
                    target=self._run_sequence,
                    args=(generation, frames),
                    daemon=True,
                    name="roscar_arm_sequence",
                ).start()
                self._publish_status(True, f"已开始动作组，共 {len(frames)} 帧")
            else:
                raise ValueError(f"unsupported arm command type: {kind}")
        except (OSError, ValueError, serial.SerialException) as exc:
            self._close()
            self._publish_status(False, f"机械臂指令失败：{exc}")

    def _run_sequence(self, generation, frames) -> None:
        try:
            for index, frame in enumerate(frames, start=1):
                with self._lock:
                    if generation != self._motion_generation:
                        return
                    self._write(pose_command(frame["pulses"], frame["duration_ms"]))
                deadline = time.monotonic() + frame["duration_ms"] / 1000.0
                while time.monotonic() < deadline:
                    with self._lock:
                        if generation != self._motion_generation:
                            return
                    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
                self._publish_status(True, f"动作组进度 {index}/{len(frames)}")
            self._publish_status(True, "动作组执行完成")
        except (OSError, serial.SerialException) as exc:
            self._close()
            self._publish_status(False, f"动作组执行失败：{exc}")

    def _run_action(self, action, _request, response):
        command = command_for(action)
        self._new_motion()
        try:
            self._write(command)
            response.success = True
            response.message = (
                f"sent {command.decode('ascii')}; board has no motion feedback"
            )
        except (OSError, serial.SerialException) as exc:
            self._close()
            response.success = False
            response.message = str(exc)
        return response

    def _close(self) -> None:
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                except serial.SerialException:
                    pass
            self._port = None

    def destroy_node(self) -> bool:
        with self._lock:
            self._motion_generation += 1
            self._close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmDriverNode()
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
