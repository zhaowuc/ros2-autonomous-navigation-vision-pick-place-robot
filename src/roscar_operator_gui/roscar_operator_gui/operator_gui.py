import os
import math
import signal
import sys
import time

from PyQt5.QtCore import QEvent, QProcess, QTimer, Qt
from PyQt5.QtGui import QBrush, QColor, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .arm_action_store import ArmActionStore
from .models import POINT_TYPE_TARGET, POINT_TYPE_WAYPOINT
from .point_store import PointStore
from .ros_worker import RosWorker


PROFILE_SPEEDS = {'low': 0.06, 'normal': 0.10, 'fast': 0.20}
MOTION_KEYS = {Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D, Qt.Key_Q, Qt.Key_E}
STATUS_TRANSLATIONS = {
    'UNKNOWN': '未知',
    'UNAVAILABLE': '不可用',
    'ERROR': '错误',
    'ACTIVE': '活动',
    'INACTIVE': '未激活',
    'NO_OUTPUT': '无输出',
    'GOAL_ACTIVE': '目标执行中',
    'IDLE': '空闲',
    'MANUAL': '手动控制',
    'NAV2': 'Nav2 导航',
    'LOW': '低速',
    'NORMAL': '标准',
    'FAST': '高速',
    'ACCEPTED': '已接受',
    'EXECUTING': '执行中',
    'CANCELING': '正在取消',
    'SUCCEEDED': '已完成',
    'CANCELED': '已取消',
    'ABORTED': '已中止',
}
DELIVERY_SEQUENCE = ('101', '102', '起点')
DELIVERY_PICKUPS = {
    '101': (8, 13, 18_500),
    '102': (3, 7, 15_500),
}
DELIVERY_PLACE = (14, 17, 12_500)


def resolve_delivery_targets(points):
    targets = []
    for name in DELIVERY_SEQUENCE:
        matches = [
            point for point in points
            if point.name == name and point.point_type == POINT_TYPE_TARGET
        ]
        if len(matches) != 1:
            raise ValueError(f'需要唯一的目标点“{name}”')
        targets.append(matches[0])
    return targets


def format_pose(pose):
    if not pose:
        return '--'
    return f"x={pose['x']:.3f}  y={pose['y']:.3f}  航向={pose['yaw']:.3f}"


def translate_status(value):
    text = str(value)
    if text.startswith('EXECUTING '):
        return '执行中 ' + text[len('EXECUTING '):]
    return STATUS_TRANSLATIONS.get(text.upper(), text)


class ArmDialog(QDialog):
    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.setWindowTitle('机械臂控制与动作组')
        self.resize(720, 620)
        self._worker = worker
        self._store = ArmActionStore()
        self._actions = []
        self._draft = []
        self._worker.arm_status.connect(self._show_status)
        self._build_ui()
        self._refresh_actions()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        actions = QGroupBox('动作组')
        actions_layout = QHBoxLayout(actions)
        self._action_list = QListWidget()
        actions_layout.addWidget(self._action_list, 1)
        action_buttons = QVBoxLayout()
        for title, callback in (
            ('执行选中动作', self._run_selected),
            ('刷新列表', self._refresh_actions),
            ('删除自定义动作', self._delete_selected),
        ):
            button = QPushButton(title)
            button.clicked.connect(callback)
            action_buttons.addWidget(button)
        action_buttons.addStretch(1)
        actions_layout.addLayout(action_buttons)
        layout.addWidget(actions, 1)

        manual = QGroupBox('6 路舵机手动姿态')
        grid = QGridLayout(manual)
        self._pulses = []
        for channel in range(6):
            editor = QSpinBox()
            editor.setRange(500, 2500)
            editor.setValue(1500)
            editor.setSingleStep(10)
            editor.setSuffix(' μs')
            self._pulses.append(editor)
            row, column = divmod(channel, 3)
            grid.addWidget(QLabel(f'S{channel:02d} / ID {channel:03d}'), row, column * 2)
            grid.addWidget(editor, row, column * 2 + 1)
        self._duration = QSpinBox()
        self._duration.setRange(100, 9999)
        self._duration.setValue(1000)
        self._duration.setSingleStep(100)
        self._duration.setSuffix(' ms')
        grid.addWidget(QLabel('运动时间'), 2, 0)
        grid.addWidget(self._duration, 2, 1)
        send_pose = QPushButton('发送当前姿态')
        send_pose.clicked.connect(self._send_pose)
        record = QPushButton('记录为动作组下一帧')
        record.clicked.connect(self._record_frame)
        grid.addWidget(send_pose, 2, 2, 1, 2)
        grid.addWidget(record, 2, 4, 1, 2)
        layout.addWidget(manual)

        draft = QGroupBox('新动作组草稿')
        draft_layout = QHBoxLayout(draft)
        self._draft_status = QLabel('当前 0 帧')
        draft_layout.addWidget(self._draft_status)
        for title, callback in (
            ('撤销末帧', self._undo_frame),
            ('清空草稿', self._clear_draft),
            ('保存动作组', self._save_draft),
        ):
            button = QPushButton(title)
            button.clicked.connect(callback)
            draft_layout.addWidget(button)
        layout.addWidget(draft)

        emergency = QPushButton('机械臂急停')
        emergency.setStyleSheet('font-weight: bold; color: #b00020;')
        emergency.clicked.connect(self._worker.stop_arm)
        layout.addWidget(emergency)
        self._status = QLabel('先确认机械臂周围无人，再执行动作。')
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

    def _show_status(self, text):
        self._status.setText(str(text))

    def _refresh_actions(self):
        try:
            self._actions = self._store.load()
            self._action_list.clear()
            for action in self._actions:
                if action['kind'] == 'stored':
                    detail = (
                        f"板内 G{action['start']:04d}"
                        if action['start'] == action['end'] else
                        f"板内 G{action['start']:04d}–G{action['end']:04d}"
                    )
                else:
                    detail = f"自定义 {len(action['frames'])} 帧"
                item = QListWidgetItem(f"{action['name']}  [{detail}]")
                item.setData(Qt.UserRole, action['id'])
                self._action_list.addItem(item)
        except Exception as exc:
            self._show_status(f'动作组加载失败：{exc}')

    def _selected_action(self):
        item = self._action_list.currentItem()
        if item is None:
            self._show_status('请先选择动作组')
            return None
        action_id = item.data(Qt.UserRole)
        return next((value for value in self._actions if value['id'] == action_id), None)

    def _run_selected(self):
        action = self._selected_action()
        if action is None:
            return
        if action['kind'] == 'stored':
            payload = {
                'type': 'stored',
                'start': action['start'],
                'end': action['end'],
                'repeat': action['repeat'],
            }
        else:
            payload = {'type': 'sequence', 'frames': action['frames']}
        self._worker.send_arm_command(payload)
        self._show_status(f"已请求执行“{action['name']}”")

    def _delete_selected(self):
        action = self._selected_action()
        if action is None:
            return
        if action['kind'] != 'sequence':
            self._show_status('板内动作不能在 GUI 中删除')
            return
        confirm = QMessageBox.question(
            self,
            '删除动作组',
            f"确定删除“{action['name']}”吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm != QMessageBox.Yes:
            return
        try:
            self._store.delete(action['id'])
            self._refresh_actions()
        except Exception as exc:
            self._show_status(f'动作组删除失败：{exc}')

    def _pose(self):
        return {
            'pulses': [editor.value() for editor in self._pulses],
            'duration_ms': self._duration.value(),
        }

    def _send_pose(self):
        self._worker.send_arm_command({'type': 'pose', **self._pose()})
        self._show_status('已请求发送当前 6 舵机姿态')

    def _record_frame(self):
        if len(self._draft) >= 100:
            self._show_status('一个动作组最多 100 帧')
            return
        self._draft.append(self._pose())
        self._draft_status.setText(f'当前 {len(self._draft)} 帧')

    def _undo_frame(self):
        if self._draft:
            self._draft.pop()
        self._draft_status.setText(f'当前 {len(self._draft)} 帧')

    def _clear_draft(self):
        self._draft.clear()
        self._draft_status.setText('当前 0 帧')

    def _save_draft(self):
        if not self._draft:
            self._show_status('请先记录至少一帧')
            return
        name, accepted = QInputDialog.getText(self, '保存动作组', '动作组名称：')
        if not accepted:
            return
        try:
            self._store.add(name, self._draft)
            self._clear_draft()
            self._refresh_actions()
            self._show_status(f'动作组“{name.strip()}”已保存')
        except Exception as exc:
            self._show_status(f'动作组保存失败：{exc}')


class OperatorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROSCAR ROS 2 智能车操作台')
        self.resize(1100, 900)
        self.setFocusPolicy(Qt.StrongFocus)
        self._pressed = set()
        self._profile = 'normal'
        self._map_yaml = os.path.expanduser('~/roscar_maps/active.yaml')
        self._point_store = PointStore()
        self._points = []
        self._rviz = QProcess(self)
        self._gemini = QProcess(self)
        self._gemini.setProcessChannelMode(QProcess.MergedChannels)
        self._gemini.finished.connect(self._gemini_finished)
        self._gemini.errorOccurred.connect(self._gemini_error)
        self._alignment_pending = False
        self._alignment_stopping = False
        self._navigation_arrived = False
        self._delivery_active = False
        self._delivery_stage = ''
        self._delivery_targets = []
        self._delivery_index = 0
        self._last_camera_frame = {'astra': 0.0, 'gemini': 0.0}
        self._arm_dialog = None
        self._worker = RosWorker(self)
        self._worker.status_changed.connect(self._status)
        self._worker.pose_snapshot.connect(self._save_pose)
        self._worker.configuration.connect(self._configuration)
        self._worker.error.connect(self._error)
        self._worker.camera_image.connect(self._camera_image)
        self._worker.alignment_status.connect(self._alignment_status)
        self._worker.navigation_arrived.connect(self._navigation_arrival)
        self._worker.arm_status.connect(self._delivery_arm_status)
        self._build_ui()
        QApplication.instance().installEventFilter(self)
        self._manual_timer = QTimer(self)
        self._manual_timer.setInterval(50)
        self._manual_timer.timeout.connect(self._publish_manual)
        self._manual_timer.start()
        self._camera_timer = QTimer(self)
        self._camera_timer.setInterval(500)
        self._camera_timer.timeout.connect(self._expire_camera_views)
        self._camera_timer.start()
        self._worker.start()
        self._refresh_points()

    def _build_ui(self):
        root = QWidget(self)
        layout = QVBoxLayout(root)

        status_box = QGroupBox('ROS 2 运行状态')
        grid = QGridLayout(status_box)
        self._status_labels = {}
        status_items = (
            ('graph_connected', 'ROS 通信图'),
            ('base_state', '底盘状态'),
            ('base_fault', '底盘故障'),
            ('nav_lifecycle', 'Nav2 生命周期'),
            ('odom', '里程计位姿'),
            ('map_pose', '地图位姿'),
            ('control_source', '控制来源'),
            ('controller_mode', '控制器模式'),
            ('collision_monitor', '碰撞监控'),
            ('speed_profile', '速度档位'),
            ('actual_linear_speed', '实际平移速度'),
            ('navigation', '导航任务'),
        )
        for index, (key, title) in enumerate(status_items):
            label = QLabel('--')
            self._status_labels[key] = label
            row = index % 6
            column = (index // 6) * 2
            grid.addWidget(QLabel(title + ':'), row, column)
            grid.addWidget(label, row, column + 1)
        layout.addWidget(status_box)

        controls = QGroupBox('运动控制')
        grid = QGridLayout(controls)
        self._speed = QComboBox()
        self._speed.addItem('低速 0.06 m/s', 'low')
        self._speed.addItem('标准 0.10 m/s', 'normal')
        self._speed.addItem('高速 0.20 m/s', 'fast')
        self._speed.setCurrentIndex(1)
        self._speed.currentIndexChanged.connect(self._speed_changed)
        grid.addWidget(QLabel('行驶速度'), 0, 0)
        grid.addWidget(self._speed, 0, 1)
        stop = QPushButton('停止 / 空格键')
        stop.clicked.connect(self._stop_motion)
        estop = QPushButton('触发急停')
        estop.clicked.connect(lambda: self._worker.set_estop(True))
        clear = QPushButton('解除急停')
        clear.clicked.connect(lambda: self._worker.set_estop(False))
        recover = QPushButton('清除底盘故障')
        recover.clicked.connect(self._recover_base)
        grid.addWidget(stop, 0, 2)
        grid.addWidget(estop, 0, 3)
        grid.addWidget(clear, 0, 4)
        grid.addWidget(recover, 0, 5)
        grid.addWidget(QLabel('按住 W/S/A/D/Q/E 控制运动，松开后立即发送零速度。'), 1, 0, 1, 6)
        layout.addWidget(controls)

        cameras = QGroupBox('深度相机')
        camera_layout = QHBoxLayout(cameras)
        self._astra_view = self._camera_label('Astra Pro 等待画面')
        self._gemini_view = self._camera_label('Gemini Pro 按需启动')
        camera_layout.addWidget(self._astra_view)
        camera_layout.addWidget(self._gemini_view)
        layout.addWidget(cameras)

        points_box = QGroupBox('点位标定与路线')
        points_layout = QHBoxLayout(points_box)
        self._point_list = QListWidget()
        points_layout.addWidget(self._point_list, 2)
        buttons = QVBoxLayout()
        for title, callback in (
            ('刷新', self._refresh_points),
            ('保存当前位置', self._request_save),
            ('编辑点位', self._edit),
            ('上移导航顺序', lambda: self._move_selected(-1)),
            ('下移导航顺序', lambda: self._move_selected(1)),
            ('删除', self._delete),
            ('按导航点前往目标点', self._navigate),
            ('取消导航', self._cancel_navigation),
        ):
            button = QPushButton(title)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        buttons.addStretch(1)
        points_layout.addLayout(buttons, 1)
        layout.addWidget(points_box, 1)
        route_hint = QLabel(
            '浅绿色＝导航点（按列表顺序、进入附近即继续）；'
            '蓝色＝目标点（精确到达并对齐箭头方向）。'
        )
        layout.addWidget(route_hint)

        tools = QHBoxLayout()
        start_rviz = QPushButton('启动额外 RViz2')
        start_rviz.clicked.connect(self._start_rviz)
        stop_rviz = QPushButton('关闭额外 RViz2')
        stop_rviz.clicked.connect(self._stop_rviz)
        tools.addWidget(start_rviz)
        tools.addWidget(stop_rviz)
        arm = QPushButton('机械臂控制')
        arm.clicked.connect(self._open_arm)
        tools.addWidget(arm)
        self._alignment_button = QPushButton('视觉对齐')
        self._alignment_button.clicked.connect(self._toggle_visual_alignment)
        tools.addWidget(self._alignment_button)
        self._delivery_button = QPushButton('开始配送')
        self._delivery_button.clicked.connect(self._toggle_delivery)
        tools.addWidget(self._delivery_button)
        tools.addStretch(1)
        self._message = QLabel('系统就绪')
        tools.addWidget(self._message)
        layout.addLayout(tools)
        self.setCentralWidget(root)

    @staticmethod
    def _camera_label(text):
        label = QLabel(text)
        label.setFixedSize(320, 240)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(
            'background: #202124; color: #d0d0d0; border: 1px solid #555;'
        )
        return label

    def _configuration(self, value):
        self._point_store = PointStore(value['points_file'])
        self._map_yaml = value['map_yaml']
        self._refresh_points()

    def _status(self, value):
        for key, label in self._status_labels.items():
            item = value.get(key)
            if key in ('odom', 'map_pose'):
                item = format_pose(item)
            elif isinstance(item, bool):
                item = '已连接' if item else '未连接'
            else:
                item = translate_status(item)
            label.setText(str(item if item is not None else '--'))
        if (
            self._delivery_active
            and self._delivery_stage == 'navigate'
            and str(value.get('navigation', '')).upper()
            in ('ABORTED', 'CANCELED', 'ERROR')
        ):
            self._cancel_delivery('配送中止：导航未成功')

    def _error(self, text):
        self._message.setText(text)

    def _camera_image(self, name, data):
        pixmap = QPixmap()
        if not pixmap.loadFromData(data, 'JPG'):
            return
        label = self._astra_view if name == 'astra' else self._gemini_view
        self._last_camera_frame[name] = time.monotonic()
        label.setPixmap(pixmap.scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def _expire_camera_views(self):
        now = time.monotonic()
        for name, label, text in (
            ('astra', self._astra_view, 'Astra Pro 画面超时'),
            ('gemini', self._gemini_view, 'Gemini Pro 按需启动'),
        ):
            last = self._last_camera_frame[name]
            if last and now - last > 1.5:
                self._last_camera_frame[name] = 0.0
                label.clear()
                label.setText(text)

    def _navigation_arrival(self, arrived):
        self._navigation_arrived = bool(arrived)
        if not arrived or not self._delivery_active or self._delivery_stage != 'navigate':
            return
        point = self._delivery_targets[self._delivery_index]
        if point.name == '起点':
            self._delivery_active = False
            self._delivery_stage = ''
            self._delivery_button.setText('开始配送')
            self._message.setText('配送完成：已返回起点')
            return
        self._delivery_stage = 'front_detect'
        self._worker.send_arm_command({
            'type': 'stored', 'start': 2, 'end': 2, 'repeat': 1,
        })
        self._message.setText(f'已到达 {point.name}：机械臂执行前方检测')
        QTimer.singleShot(3500, self._delivery_start_alignment)

    def _alignment_status(self, payload):
        state = str(payload.get('state') or '未知')
        message = str(payload.get('message') or '')
        depth = payload.get('depth')
        suffix = f'，深度 {float(depth):.3f} m' if depth is not None else ''
        self._message.setText(f'视觉对齐：{state} — {message}{suffix}')
        if state == '已对齐':
            self._alignment_button.setText('已对齐 / 关闭 Gemini')
            if self._delivery_active and self._delivery_stage == 'align':
                point = self._delivery_targets[self._delivery_index]
                self._stop_visual_alignment()
                start, end, wait_ms = DELIVERY_PICKUPS[point.name]
                self._delivery_stage = 'pickup'
                self._worker.send_arm_command({
                    'type': 'stored', 'start': start, 'end': end, 'repeat': 1,
                })
                self._message.setText(f'{point.name} 对齐完成：正在拾取药包')
                QTimer.singleShot(wait_ms, self._delivery_place)
        elif state == '超时' and self._delivery_active:
            self._cancel_delivery('配送中止：视觉对齐超时')

    def _toggle_visual_alignment(self):
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()
            return
        note = (
            '已确认导航到达。' if self._navigation_arrived else
            '当前 GUI 会话未记录导航到达。'
        )
        answer = QMessageBox.question(
            self,
            '启动视觉对齐',
            note + '\n请确认车体和机械臂周围无人、红砖平台前方无杂物。\n'
            '将执行“前方检测”动作，然后以最高 0.060 m/s 微调车体。是否继续？',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self._stop_motion()
        self._worker.cancel_navigation()
        self._worker.send_arm_command({
            'type': 'stored', 'start': 2, 'end': 2, 'repeat': 1,
        })
        self._alignment_pending = True
        self._alignment_button.setText('取消视觉对齐')
        self._message.setText('机械臂正在执行“前方检测”，3.5 秒后启动 Gemini')
        QTimer.singleShot(3500, self._start_gemini)

    def _start_gemini(self):
        if not self._alignment_pending:
            return
        self._alignment_pending = False
        self._alignment_stopping = False
        self._gemini.start(
            'setsid', [
                'ros2', 'launch', 'roscar_depth_camera',
                'gemini_alignment.launch.py',
            ]
        )
        self._alignment_button.setText('停止视觉对齐')
        self._message.setText('Gemini 正在启动；目标稳定后才会低速微调')

    def _stop_visual_alignment(self):
        self._alignment_pending = False
        self._alignment_stopping = True
        if self._gemini.state() != QProcess.NotRunning:
            try:
                os.killpg(int(self._gemini.processId()), signal.SIGTERM)
            except (OSError, ValueError):
                self._gemini.terminate()
            if not self._gemini.waitForFinished(2000):
                try:
                    os.killpg(int(self._gemini.processId()), signal.SIGKILL)
                except (OSError, ValueError):
                    self._gemini.kill()
                self._gemini.waitForFinished(1000)
        self._alignment_button.setText('视觉对齐')
        self._last_camera_frame['gemini'] = 0.0
        self._gemini_view.clear()
        self._gemini_view.setText('Gemini Pro 按需启动')
        self._message.setText('视觉对齐已停止')

    def _gemini_finished(self, exit_code, _exit_status):
        self._alignment_button.setText('视觉对齐')
        self._last_camera_frame['gemini'] = 0.0
        self._gemini_view.clear()
        self._gemini_view.setText('Gemini Pro 按需启动')
        if not self._alignment_stopping and exit_code:
            output = bytes(self._gemini.readAllStandardOutput()).decode(
                'utf-8', errors='replace'
            )[-300:]
            self._message.setText(f'Gemini 异常退出：{output or exit_code}')
        if (
            not self._alignment_stopping
            and self._delivery_active
            and self._delivery_stage == 'align'
        ):
            self._cancel_delivery('配送中止：Gemini 意外退出')
        self._alignment_stopping = False

    def _gemini_error(self, _error):
        if self._alignment_stopping:
            return
        self._message.setText(f'Gemini 启动失败：{self._gemini.errorString()}')
        if self._delivery_active:
            self._cancel_delivery('配送中止：Gemini 启动失败')

    def _delivery_arm_status(self, text):
        if self._delivery_active and any(
            marker in str(text) for marker in ('失败', '未在线', '被拒绝')
        ):
            self._cancel_delivery(f'配送中止：{text}')

    def _toggle_delivery(self):
        if self._delivery_active:
            self._cancel_delivery('配送已手动停止')
            return
        try:
            targets = resolve_delivery_targets(self._points)
        except ValueError as exc:
            self._error(str(exc))
            return
        answer = QMessageBox.question(
            self,
            '开始配送',
            '将自动执行：起点 → 101 → 102 → 起点。\n'
            '101 左侧拾取、102 右侧拾取；两站均会视觉对齐、放置药包并初始化。\n'
            '请确认车体与机械臂周围无人、无杂物。',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()
        self._delivery_active = True
        self._delivery_targets = targets
        self._delivery_index = 0
        self._delivery_button.setText('停止配送')
        self._delivery_navigate()

    def _delivery_navigate(self):
        if not self._delivery_active:
            return
        point = self._delivery_targets[self._delivery_index]
        waypoints = [
            value.to_mapping() for value in self._points
            if value.point_type == POINT_TYPE_WAYPOINT
        ]
        self._delivery_stage = 'navigate'
        self._worker.navigate_route(waypoints, point.to_mapping())
        self._message.setText(f'配送：机械臂初始化后前往 {point.name}')

    def _delivery_start_alignment(self):
        if not self._delivery_active or self._delivery_stage != 'front_detect':
            return
        self._delivery_stage = 'align'
        self._alignment_pending = True
        self._start_gemini()
        point = self._delivery_targets[self._delivery_index]
        self._message.setText(f'配送 {point.name}：正在视觉对齐')

    def _delivery_place(self):
        if not self._delivery_active or self._delivery_stage != 'pickup':
            return
        start, end, wait_ms = DELIVERY_PLACE
        self._delivery_stage = 'place'
        self._worker.send_arm_command({
            'type': 'stored', 'start': start, 'end': end, 'repeat': 1,
        })
        self._message.setText('配送：正在放置药包')
        QTimer.singleShot(wait_ms, self._delivery_continue)

    def _delivery_continue(self):
        if not self._delivery_active or self._delivery_stage != 'place':
            return
        self._delivery_index += 1
        self._delivery_navigate()

    def _cancel_delivery(self, message):
        if not self._delivery_active:
            return
        self._delivery_active = False
        self._delivery_stage = ''
        self._delivery_targets = []
        self._delivery_index = 0
        self._delivery_button.setText('开始配送')
        self._worker.cancel_navigation()
        self._worker.stop_arm()
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()
        self._message.setText(message)

    def _cancel_navigation(self):
        if self._delivery_active:
            self._cancel_delivery('配送已取消')
        else:
            self._worker.cancel_navigation()

    def _speed_changed(self):
        self._profile = str(self._speed.currentData())
        self._worker.set_speed_profile(self._profile)

    def _manual_vector(self):
        speed = PROFILE_SPEEDS[self._profile]
        vx = speed * ((Qt.Key_W in self._pressed) - (Qt.Key_S in self._pressed))
        vy = speed * ((Qt.Key_A in self._pressed) - (Qt.Key_D in self._pressed))
        wz = 0.25 * ((Qt.Key_Q in self._pressed) - (Qt.Key_E in self._pressed))
        return vx, vy, wz

    def _publish_manual(self):
        if not self._pressed.intersection(MOTION_KEYS):
            return
        if self._delivery_active:
            self._cancel_delivery('手动控制已中止配送')
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()
        self._worker.set_manual(*self._manual_vector(), active=True)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self._stop_motion()
            event.accept()
            return
        if event.key() in MOTION_KEYS and not event.isAutoRepeat():
            self._pressed.add(event.key())
            self._publish_manual()
            event.accept()
            return
        super().keyPressEvent(event)

    def eventFilter(self, watched, event):
        event_type = event.type()
        if event_type in (QEvent.ApplicationDeactivate, QEvent.WindowDeactivate):
            if self._pressed:
                self._stop_motion()
            return False
        if event_type not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False
        key = event.key()
        if key == Qt.Key_Space and event_type == QEvent.KeyPress:
            self._stop_motion()
            return True
        if key not in MOTION_KEYS or QApplication.activeModalWidget() is not None:
            return False
        focus = QApplication.focusWidget()
        if isinstance(focus, (QLineEdit, QDoubleSpinBox)):
            return False
        if event.isAutoRepeat():
            return True
        if event_type == QEvent.KeyPress:
            self._pressed.add(key)
            self._publish_manual()
        else:
            self._pressed.discard(key)
            if self._pressed.intersection(MOTION_KEYS):
                self._publish_manual()
            else:
                self._worker.stop_manual()
        return True

    def keyReleaseEvent(self, event):
        if event.key() in MOTION_KEYS and not event.isAutoRepeat():
            self._pressed.discard(event.key())
            if self._pressed.intersection(MOTION_KEYS):
                self._publish_manual()
            else:
                self._worker.stop_manual()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _stop_motion(self):
        self._pressed.clear()
        self._worker.stop_manual()
        if self._delivery_active:
            self._cancel_delivery('配送已停止')
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()

    def _recover_base(self):
        self._stop_motion()
        self._worker.recover_base()

    def _refresh_points(self):
        try:
            self._points = self._point_store.load()
            self._point_list.clear()
            for point in self._points:
                is_waypoint = point.point_type == POINT_TYPE_WAYPOINT
                point_kind = '导航点' if is_waypoint else '目标点'
                detail = (
                    f'x={point.x:.3f}, y={point.y:.3f}'
                    if is_waypoint else
                    f'x={point.x:.3f}, y={point.y:.3f}, '
                    f'车头={math.degrees(point.yaw):.1f}°'
                )
                item = QListWidgetItem(
                    f'[{point_kind}] {point.name}  ({detail})'
                )
                item.setForeground(QBrush(QColor(
                    '#66d98b' if is_waypoint else '#3f7fff'
                )))
                item.setData(Qt.UserRole, point.point_id)
                self._point_list.addItem(item)
            self._worker.update_points([
                point.to_mapping() for point in self._points
            ])
        except Exception as exc:
            self._error(f'点位加载失败：{exc}')

    def _selected_point(self):
        item = self._point_list.currentItem()
        if item is None:
            self._error('请先选择一个点位')
            return None
        point_id = item.data(Qt.UserRole)
        return next((point for point in self._points if point.point_id == point_id), None)

    def _request_save(self):
        self._worker.request_pose()

    def _save_pose(self, pose):
        dialog = QDialog(self)
        dialog.setWindowTitle('保存点位')
        form = QFormLayout(dialog)
        name = QLineEdit()
        point_type = self._point_type_editor(POINT_TYPE_TARGET)
        form.addRow('点位名称：', name)
        form.addRow('点位类型：', point_type)
        form.addRow(QLabel('目标点保存车头方向；导航点仅用于引导路线经过附近。'))
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText('保存')
        buttons.button(QDialogButtonBox.Cancel).setText('取消')
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            self._point_store.add(
                name.text(), pose['x'], pose['y'], pose['yaw'],
                pose.get('frame_id', 'map'), self._map_yaml,
                point_type=str(point_type.currentData()),
            )
            self._refresh_points()
        except Exception as exc:
            self._error(f'点位保存失败：{exc}')

    def _edit(self):
        point = self._selected_point()
        if point is None:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle('编辑点位')
        form = QFormLayout(dialog)
        name = QLineEdit(point.name)
        point_type = self._point_type_editor(point.point_type)
        x_value = self._coordinate_editor(point.x)
        y_value = self._coordinate_editor(point.y)
        heading = QDoubleSpinBox()
        heading.setRange(-180.0, 180.0)
        heading.setDecimals(2)
        heading.setSingleStep(1.0)
        heading.setSuffix('°')
        heading.setValue(math.degrees(point.yaw))
        form.addRow('点位名称：', name)
        form.addRow('点位类型：', point_type)
        form.addRow('X 坐标（米）：', x_value)
        form.addRow('Y 坐标（米）：', y_value)
        form.addRow('车头航向：', heading)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText('保存')
        buttons.button(QDialogButtonBox.Cancel).setText('取消')
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec_() != QDialog.Accepted:
            return
        try:
            self._point_store.edit(
                point.point_id,
                name.text(),
                x_value.value(),
                y_value.value(),
                math.radians(heading.value()),
                point_type=str(point_type.currentData()),
            )
            self._refresh_points()
        except Exception as exc:
            self._error(f'点位修改失败：{exc}')

    @staticmethod
    def _coordinate_editor(value):
        editor = QDoubleSpinBox()
        editor.setRange(-1000.0, 1000.0)
        editor.setDecimals(4)
        editor.setSingleStep(0.05)
        editor.setValue(float(value))
        return editor

    @staticmethod
    def _point_type_editor(value):
        editor = QComboBox()
        editor.addItem('目标点（严格到达并对齐车头）', POINT_TYPE_TARGET)
        editor.addItem('导航点（路线经过附近）', POINT_TYPE_WAYPOINT)
        editor.setCurrentIndex(1 if value == POINT_TYPE_WAYPOINT else 0)
        return editor

    def _move_selected(self, offset):
        point = self._selected_point()
        if point is None:
            return
        if point.point_type != POINT_TYPE_WAYPOINT:
            self._error('只有导航点需要调整路线顺序')
            return
        try:
            waypoints = [
                value for value in self._points
                if value.point_type == POINT_TYPE_WAYPOINT
            ]
            route_index = next(
                index for index, value in enumerate(waypoints)
                if value.point_id == point.point_id
            )
            destination_index = max(
                0, min(len(waypoints) - 1, route_index + int(offset))
            )
            if destination_index == route_index:
                return
            current_row = next(
                index for index, value in enumerate(self._points)
                if value.point_id == point.point_id
            )
            destination_row = next(
                index for index, value in enumerate(self._points)
                if value.point_id == waypoints[destination_index].point_id
            )
            self._point_store.move(point.point_id, destination_row - current_row)
            self._refresh_points()
            for row in range(self._point_list.count()):
                item = self._point_list.item(row)
                if item.data(Qt.UserRole) == point.point_id:
                    self._point_list.setCurrentRow(row)
                    break
        except Exception as exc:
            self._error(f'导航顺序调整失败：{exc}')

    def _delete(self):
        point = self._selected_point()
        if point is None:
            return
        confirm = QMessageBox(self)
        confirm.setWindowTitle('删除点位')
        confirm.setText(f'确定删除“{point.name}”吗？')
        confirm.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        confirm.button(QMessageBox.Yes).setText('删除')
        confirm.button(QMessageBox.No).setText('取消')
        if confirm.exec_() == QMessageBox.Yes:
            try:
                self._point_store.delete(point.point_id)
                self._refresh_points()
            except Exception as exc:
                self._error(f'点位删除失败：{exc}')

    def _get_text(self, title, label, value=''):
        dialog = QInputDialog(self)
        dialog.setWindowTitle(title)
        dialog.setLabelText(label)
        dialog.setTextValue(value)
        dialog.setOkButtonText('确定')
        dialog.setCancelButtonText('取消')
        accepted = bool(dialog.exec_())
        return dialog.textValue(), accepted

    def _navigate(self):
        point = self._selected_point()
        if point is None:
            return
        if point.point_type != POINT_TYPE_TARGET:
            self._error('请选择一个蓝色目标点作为最终目的地')
            return
        if self._delivery_active:
            self._cancel_delivery('手动导航已中止配送')
        if self._alignment_pending or self._gemini.state() != QProcess.NotRunning:
            self._stop_visual_alignment()
        waypoints = [
            value.to_mapping() for value in self._points
            if value.point_type == POINT_TYPE_WAYPOINT
        ]
        self._worker.navigate_route(waypoints, point.to_mapping())
        self._message.setText(
            f'已发送路线：{len(waypoints)} 个导航点 → 目标点“{point.name}”'
        )

    def _start_rviz(self):
        if self._rviz.state() != QProcess.NotRunning:
            return
        self._rviz.start('rviz2', [])

    def _stop_rviz(self):
        if self._rviz.state() != QProcess.NotRunning:
            self._rviz.terminate()
            if not self._rviz.waitForFinished(1500):
                self._rviz.kill()

    def _open_arm(self):
        if self._arm_dialog is None:
            self._arm_dialog = ArmDialog(self._worker, self)
        self._arm_dialog.show()
        self._arm_dialog.raise_()
        self._arm_dialog.activateWindow()

    def closeEvent(self, event):
        self._stop_motion()
        self._stop_visual_alignment()
        self._worker.stop_arm()
        self._worker.cancel_navigation()
        self._worker.stop_worker()
        self._worker.wait(3000)
        self._stop_rviz()
        event.accept()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    os.environ.setdefault('QT_AUTO_SCREEN_SCALE_FACTOR', '1')
    application = QApplication(sys.argv)
    window = OperatorWindow()
    window.show()
    return application.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
