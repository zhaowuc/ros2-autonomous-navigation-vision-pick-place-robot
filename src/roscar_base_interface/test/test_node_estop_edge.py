import importlib
import sys
import threading
import types

from roscar_base_interface.backends import (
    BackendCommand,
    ControlDeliveryState,
    SerialBackend,
    SerialLinkState,
)
from roscar_base_interface.protocol import CommandFlag


def _install_module(monkeypatch, name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    monkeypatch.setitem(sys.modules, name, module)
    return module


def _import_node_without_ros(monkeypatch):
    class Placeholder:
        pass

    class Trigger:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.success = False
                self.message = ''

    class QoSProfile:
        def __init__(self, **_kwargs):
            pass

    _install_module(monkeypatch, 'rclpy', init=lambda: None, shutdown=lambda: None)
    _install_module(
        monkeypatch,
        'rclpy.clock',
        Clock=Placeholder,
        ClockType=types.SimpleNamespace(STEADY_TIME=1),
    )
    _install_module(
        monkeypatch,
        'rclpy.executors',
        ExternalShutdownException=RuntimeError,
    )
    _install_module(monkeypatch, 'rclpy.node', Node=Placeholder)
    _install_module(
        monkeypatch,
        'rclpy.qos',
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
        HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
        QoSProfile=QoSProfile,
        ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2),
    )

    for package, message_names in (
        ('c50c_interfaces', ('BaseStatus', 'WheelState')),
        ('diagnostic_msgs', ('DiagnosticArray', 'DiagnosticStatus', 'KeyValue')),
        ('geometry_msgs', ('Twist',)),
        ('nav_msgs', ('Odometry',)),
        ('sensor_msgs', ('BatteryState',)),
        ('std_msgs', ('Bool',)),
    ):
        _install_module(monkeypatch, package)
        _install_module(
            monkeypatch,
            f'{package}.msg',
            **{name: Placeholder for name in message_names},
        )
    _install_module(monkeypatch, 'std_srvs')
    _install_module(monkeypatch, 'std_srvs.srv', Trigger=Trigger)

    monkeypatch.delitem(
        sys.modules,
        'roscar_base_interface.base_interface_node',
        raising=False,
    )
    module_name = 'roscar_base_interface.base_interface_node'
    node_module = importlib.import_module(module_name)
    # Do not leave a module bound to temporary ROS stubs for later tests.
    sys.modules.pop(module_name, None)
    return node_module, Trigger


def test_node_estop_true_false_pulse_cancels_requests_before_next_tick(monkeypatch):
    node_module, trigger_type = _import_node_without_ros(monkeypatch)
    backend = SerialBackend(serial_factory=lambda **_kwargs: None)
    backend.state = SerialLinkState.READY
    backend.set_command(
        BackendCommand(7, 0.0, 0.0, 0.0, int(CommandFlag.REARM), 1.0)
    )
    assert backend.control_delivery_state == ControlDeliveryState.QUEUED

    node = object.__new__(node_module.BaseInterfaceNode)
    node.lock = threading.Lock()
    node.backend = backend
    node.estop_active = False
    node.pending_flags = int(CommandFlag.REARM | CommandFlag.CLEAR_FAULT)

    # Both reliable samples arrive between two control ticks.
    node._estop_callback(types.SimpleNamespace(data=True))
    assert node.pending_flags == 0
    assert backend.control_delivery_state == ControlDeliveryState.FAILED
    assert backend.control_failed_flags & int(CommandFlag.REARM)

    first_error = backend.control_last_error
    node._estop_callback(types.SimpleNamespace(data=True))
    assert backend.control_last_error == first_error
    assert backend.control_failed_flags == int(CommandFlag.REARM)

    rejected = trigger_type.Response()
    node._rearm_service(trigger_type.Request(), rejected)
    assert not rejected.success
    assert node.pending_flags == 0

    node._estop_callback(types.SimpleNamespace(data=False))
    assert not node.estop_active
    assert node.pending_flags == 0
    assert backend.control_pending_flags == 0

    # ESTOP release invalidates serial authority.  A control transaction must
    # remain rejected until a new feedback/fresh-zero/exact-ACK handshake has
    # restored READY.
    after_release = trigger_type.Response()
    node._rearm_service(trigger_type.Request(), after_release)
    assert not after_release.success
    assert node.pending_flags == 0

    backend.transport = object()
    backend.state = SerialLinkState.READY
    accepted = trigger_type.Response()
    node._rearm_service(trigger_type.Request(), accepted)
    assert accepted.success
    assert node.pending_flags == int(CommandFlag.REARM)


def test_node_supplies_fresh_startup_zero_without_nav2(monkeypatch):
    node_module, _ = _import_node_without_ros(monkeypatch)

    class Backend:
        diagnostic_summary = ''

        def set_command(self, command):
            self.command = command

        def tick(self, _now):
            return []

    node = object.__new__(node_module.BaseInterfaceNode)
    node.lock = threading.Lock()
    node.backend = Backend()
    node.latest_command_at = 0.0
    node.latest_command_valid = False
    node.command_timeout = 0.15
    node.estop_active = False
    node.pending_flags = 0
    node.sequence = 0
    node.max_vx = node.max_vy = node.max_wz = 0.2
    node.last_status_publish = node.last_diagnostics_publish = 10.0
    node.last_diagnostic_log = 10.0
    node.diagnostics_every_control_tick = False
    node.diagnostic_period = 1.0
    node._publish_sim_command = lambda: None
    monkeypatch.setattr(node_module.time, 'monotonic', lambda: 10.0)

    node._control_tick()

    assert node.backend.command.is_zero
    assert node.backend.command.flags == 0
    assert node.backend.command.source_stamp == 10.0
