"""Cosys-AirSim mission-control desktop application."""

from __future__ import annotations

import itertools
import math
import queue
import sys
import time
from pathlib import Path

PYTHON_CLIENT_ROOT = Path(__file__).resolve().parents[1]
if str(PYTHON_CLIENT_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_CLIENT_ROOT))

import numpy as np
from PySide6.QtCore import QSettings, QThread, Signal
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from common.airsim_client import AirSimController
from missions.heart_mission import build_heart_path
from missions.square_mission import build_square_path
from navigation.grid_planner import AltitudeGridPlanner, PlannerConfig
from perception.camera_viewer import CameraViewer
from perception.lidar_viewer import LidarViewer
from ui.minimap import MiniMapWidget


class AirSimWorker(QThread):
    connection_changed = Signal(bool, str)
    telemetry_updated = Signal(dict)
    images_updated = Signal(dict)
    lidar_updated = Signal(object)
    command_completed = Signal(str)
    error_occurred = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.controller = AirSimController()
        self._commands: queue.PriorityQueue = queue.PriorityQueue()
        self._sequence = itertools.count()
        self._running = True
        self._stream_sensors = True
        self._last_sensor_error = 0.0

    def submit(self, name: str, *args, priority: int = 10) -> None:
        self._commands.put((priority, next(self._sequence), name, args))

    def stop(self) -> None:
        self._running = False

    def set_sensor_streaming(self, enabled: bool) -> None:
        self._stream_sensors = enabled

    def run(self) -> None:
        next_telemetry = 0.0
        next_sensors = 0.0
        while self._running:
            self._process_pending_commands()
            now = time.monotonic()
            if self.controller.connected and now >= next_telemetry:
                self._poll_telemetry()
                next_telemetry = now + 0.1
            if self.controller.connected and self._stream_sensors and now >= next_sensors:
                self._poll_sensors()
                next_sensors = now + 0.35
            self.msleep(20)
        self.controller.disconnect()

    def _process_pending_commands(self) -> None:
        for _ in range(5):
            try:
                _, _, name, args = self._commands.get_nowait()
            except queue.Empty:
                return
            try:
                if name == "connect":
                    self.controller.connect()
                    self.connection_changed.emit(True, "연결됨")
                elif name == "disconnect":
                    self.controller.disconnect()
                    self.connection_changed.emit(False, "연결 해제")
                elif name == "arm":
                    self.controller.arm(bool(args[0]))
                elif name == "spawn":
                    self.controller.set_spawn(float(args[0]), float(args[1]))
                elif name == "takeoff":
                    self.controller.takeoff(float(args[0]))
                elif name == "hover":
                    self.controller.hover()
                elif name == "move":
                    self.controller.move_to(*args)
                elif name == "path":
                    self.controller.move_path(*args)
                elif name == "land":
                    self.controller.land()
                elif name == "emergency":
                    self.controller.emergency_stop()
                else:
                    raise ValueError(f"알 수 없는 명령: {name}")
                if name not in {"connect", "disconnect"}:
                    self.command_completed.emit(name)
            except Exception as exc:
                if name == "connect":
                    self.connection_changed.emit(False, "연결 실패")
                self.error_occurred.emit(f"{name}: {exc}")

    def _poll_telemetry(self) -> None:
        try:
            self.telemetry_updated.emit(self.controller.telemetry())
        except Exception as exc:
            self.connection_changed.emit(False, "통신 오류")
            self.error_occurred.emit(f"텔레메트리: {exc}")
            self.controller.disconnect()

    def _poll_sensors(self) -> None:
        try:
            self.images_updated.emit(self.controller.camera_images())
            self.lidar_updated.emit(self.controller.lidar_snapshot())
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_sensor_error > 5.0:
                self.error_occurred.emit(f"센서: {exc}")
                self._last_sensor_error = now


class MissionControlWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Autonomous Drone Mission Control")
        self.resize(1500, 920)
        self.settings = QSettings("AutonomousDrone", "MissionControl")
        self._connected = False
        # The AirBase capture covers 1400 m. A 2.5 m planning grid keeps
        # full-map A* practical while retaining useful obstacle clearance.
        self.planner = AltitudeGridPlanner(
            PlannerConfig(half_extent_m=900.0, resolution_m=2.5)
        )
        self._telemetry: dict | None = None
        self._planned_path: list[tuple[float, float, float]] = []
        self._active_target: tuple[float, float, float] | None = None
        self._last_replan = 0.0
        self._spawn_xy = (0.0, 0.0)

        self.worker = AirSimWorker()
        self.worker.connection_changed.connect(self._on_connection_changed)
        self.worker.telemetry_updated.connect(self._on_telemetry)
        self.worker.images_updated.connect(self._on_images)
        self.worker.lidar_updated.connect(self._on_lidar)
        self.worker.command_completed.connect(self._on_command_completed)
        self.worker.error_occurred.connect(self._on_error)
        self.worker.start()

        self._build_ui()
        self._apply_style()
        self._restore_settings()
        self._set_controls_enabled(False)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        header = QHBoxLayout()
        title = QLabel("AUTONOMOUS DRONE · MISSION CONTROL")
        title.setObjectName("title")
        self.status_indicator = QLabel("● 연결 안 됨")
        self.status_indicator.setObjectName("disconnected")
        self.connect_button = QPushButton("AirSim 연결")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.sensor_checkbox = QCheckBox("센서 스트리밍")
        self.sensor_checkbox.setChecked(True)
        self.sensor_checkbox.toggled.connect(self.worker.set_sensor_streaming)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.sensor_checkbox)
        header.addWidget(self.status_indicator)
        header.addWidget(self.connect_button)
        root.addLayout(header)

        splitter = QSplitter()
        splitter.addWidget(self._build_control_panel())
        splitter.addWidget(self._build_sensor_panel())
        splitter.setSizes([380, 1120])
        root.addWidget(splitter)

        self.message_label = QLabel("Unreal에서 Play를 실행한 뒤 AirSim 연결을 누르세요.")
        self.message_label.setObjectName("message")
        root.addWidget(self.message_label)

    def _build_control_panel(self) -> QWidget:
        panel = QFrame()
        panel.setMaximumWidth(430)
        layout = QVBoxLayout(panel)

        flight_group = QGroupBox("비행 제어")
        flight_layout = QGridLayout(flight_group)
        self.arm_button = QPushButton("ARM")
        self.disarm_button = QPushButton("DISARM")
        self.takeoff_button = QPushButton("이륙")
        self.hover_button = QPushButton("호버링")
        self.land_button = QPushButton("착륙")
        self.emergency_button = QPushButton("긴급 정지")
        self.emergency_button.setObjectName("emergency")
        self.arm_button.clicked.connect(lambda: self.worker.submit("arm", True))
        self.disarm_button.clicked.connect(lambda: self.worker.submit("arm", False))
        self.takeoff_button.clicked.connect(self._takeoff)
        self.hover_button.clicked.connect(lambda: self.worker.submit("hover"))
        self.land_button.clicked.connect(lambda: self.worker.submit("land"))
        self.emergency_button.clicked.connect(lambda: self.worker.submit("emergency", priority=0))
        flight_layout.addWidget(self.arm_button, 0, 0)
        flight_layout.addWidget(self.disarm_button, 0, 1)
        flight_layout.addWidget(self.takeoff_button, 1, 0)
        flight_layout.addWidget(self.hover_button, 1, 1)
        flight_layout.addWidget(self.land_button, 2, 0)
        flight_layout.addWidget(self.emergency_button, 2, 1)
        layout.addWidget(flight_group)

        destination_group = QGroupBox("목적지 · NED 기준")
        destination_form = QFormLayout(destination_group)
        self.takeoff_altitude = self._spinbox(1.0, 120.0, 5.0, " m")
        self.destination_x = self._spinbox(-2000.0, 2000.0, 10.0, " m")
        self.destination_y = self._spinbox(-2000.0, 2000.0, 0.0, " m")
        self.destination_altitude = self._spinbox(1.0, 120.0, 5.0, " m")
        self.speed = self._spinbox(0.2, 20.0, 3.0, " m/s")
        destination_form.addRow("이륙 고도", self.takeoff_altitude)
        destination_form.addRow("X · 전방/북쪽", self.destination_x)
        destination_form.addRow("Y · 오른쪽/동쪽", self.destination_y)
        destination_form.addRow("목적지 고도", self.destination_altitude)
        destination_form.addRow("이동 속도", self.speed)
        map_mode_row = QHBoxLayout()
        self.spawn_select_button = QPushButton("스폰 A 선택")
        self.target_select_button = QPushButton("목표 B 선택")
        self.spawn_select_button.setCheckable(True)
        self.target_select_button.setCheckable(True)
        self.target_select_button.setChecked(True)
        self.spawn_select_button.clicked.connect(lambda: self._set_map_mode("spawn"))
        self.target_select_button.clicked.connect(lambda: self._set_map_mode("target"))
        map_mode_row.addWidget(self.spawn_select_button)
        map_mode_row.addWidget(self.target_select_button)
        destination_form.addRow("지도 클릭 모드", map_mode_row)
        self.apply_spawn_button = QPushButton("선택한 위치에 스폰 적용")
        self.apply_spawn_button.clicked.connect(self._apply_spawn)
        destination_form.addRow(self.apply_spawn_button)
        self.auto_replan_checkbox = QCheckBox("LiDAR 장애물 감지 시 자동 재탐색")
        self.auto_replan_checkbox.setChecked(True)
        destination_form.addRow(self.auto_replan_checkbox)
        self.move_button = QPushButton("A* 경로로 목표 B 이동")
        self.move_button.clicked.connect(self._move)
        destination_form.addRow(self.move_button)
        layout.addWidget(destination_group)

        mission_group = QGroupBox("패턴 미션")
        mission_layout = QHBoxLayout(mission_group)
        self.square_button = QPushButton("사각형 비행")
        self.heart_button = QPushButton("하트 비행")
        self.square_button.clicked.connect(self._square_mission)
        self.heart_button.clicked.connect(self._heart_mission)
        mission_layout.addWidget(self.square_button)
        mission_layout.addWidget(self.heart_button)
        layout.addWidget(mission_group)

        telemetry_group = QGroupBox("실시간 텔레메트리")
        telemetry_layout = QFormLayout(telemetry_group)
        self.telemetry_labels: dict[str, QLabel] = {}
        fields = [
            ("상태", "landed"),
            ("위치 X / Y", "position"),
            ("고도", "altitude"),
            ("속도", "speed"),
            ("속도 벡터", "velocity"),
            ("Roll / Pitch / Yaw", "attitude"),
        ]
        for label, key in fields:
            value = QLabel("—")
            value.setTextInteractionFlags(value.textInteractionFlags())
            self.telemetry_labels[key] = value
            telemetry_layout.addRow(label, value)
        layout.addWidget(telemetry_group)
        layout.addStretch()
        return panel

    def _build_sensor_panel(self) -> QWidget:
        tabs = QTabWidget()
        self.camera_viewer = CameraViewer()
        self.lidar_viewer = LidarViewer()
        minimap_image = PYTHON_CLIENT_ROOT / "assets" / "Minimap_AirBase.PNG"
        self.minimap = MiniMapWidget(
            half_extent_m=700.0,
            center_xy_m=(53.31, 159.39),
            background_path=str(minimap_image),
        )
        self.minimap.spawn_selected.connect(self._on_spawn_selected)
        self.minimap.target_selected.connect(self._on_target_selected)
        tabs.addTab(self.minimap, "미니맵 · A* 경로")
        tabs.addTab(self.camera_viewer, "RGB · Depth · Segmentation")
        tabs.addTab(self.lidar_viewer, "LiDAR 3D 점군")
        return tabs

    @staticmethod
    def _spinbox(minimum: float, maximum: float, value: float, suffix: str) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(1)
        widget.setSingleStep(1.0)
        widget.setValue(value)
        widget.setSuffix(suffix)
        return widget

    def _toggle_connection(self) -> None:
        if self._connected:
            self.worker.submit("disconnect", priority=1)
        else:
            self.message_label.setText("AirSim 연결 중…")
            self.worker.submit("connect", priority=1)

    def _takeoff(self) -> None:
        self.worker.submit("takeoff", self.takeoff_altitude.value())

    def _move(self) -> None:
        try:
            self._plan_and_fly(replan=False)
        except Exception as exc:
            self._on_error(f"경로계획: {exc}")

    def _set_map_mode(self, mode: str) -> None:
        self.minimap.set_selection_mode(mode)
        is_spawn = mode == "spawn"
        self.spawn_select_button.setChecked(is_spawn)
        self.target_select_button.setChecked(not is_spawn)
        self.message_label.setText(
            "미니맵에서 스폰 A를 클릭하세요."
            if is_spawn
            else "미니맵에서 목표 B를 클릭하세요."
        )

    def _on_spawn_selected(self, x_m: float, y_m: float) -> None:
        self._spawn_xy = (x_m, y_m)
        self.message_label.setText(
            f"스폰 A 선택: X={x_m:.1f}, Y={y_m:.1f}m · 적용 버튼을 누르세요."
        )

    def _on_target_selected(self, x_m: float, y_m: float) -> None:
        self.destination_x.setValue(x_m)
        self.destination_y.setValue(y_m)
        self._active_target = None
        self.message_label.setText(f"목표 B 선택: X={x_m:.1f}, Y={y_m:.1f}m")

    def _apply_spawn(self) -> None:
        self.worker.submit("spawn", self._spawn_xy[0], self._spawn_xy[1], priority=2)

    def _plan_and_fly(self, replan: bool) -> None:
        if self._telemetry is None:
            raise RuntimeError("드론 위치를 아직 받지 못했습니다.")
        target = (
            self.destination_x.value(),
            self.destination_y.value(),
            self.destination_altitude.value(),
        )
        start = (float(self._telemetry["x"]), float(self._telemetry["y"]))
        path = self.planner.plan(
            start,
            (target[0], target[1]),
            target[2],
            float(self._telemetry["altitude"]),
        )
        self._planned_path = path
        self._active_target = target
        self.minimap.set_target(target[0], target[1])
        self.minimap.set_path(path)
        if replan:
            self.worker.submit("emergency", priority=0)
        self.worker.submit("path", path, self.speed.value(), priority=3)
        cruise = max(point[2] for point in path)
        self.message_label.setText(
            f"{'재탐색' if replan else '경로 생성'} 완료: "
            f"웨이포인트 {len(path)}개, 최고 {cruise:.1f}m"
        )

    def _square_mission(self) -> None:
        points = build_square_path(
            self.destination_x.value(),
            self.destination_y.value(),
            self.destination_altitude.value(),
        )
        self.worker.submit("path", points, self.speed.value())

    def _heart_mission(self) -> None:
        points = build_heart_path(
            self.destination_x.value(),
            self.destination_y.value(),
            self.destination_altitude.value(),
        )
        self.worker.submit("path", points, self.speed.value())

    def _on_connection_changed(self, connected: bool, message: str) -> None:
        self._connected = connected
        self.status_indicator.setText(f"● {message}")
        self.status_indicator.setObjectName("connected" if connected else "disconnected")
        self.status_indicator.style().unpolish(self.status_indicator)
        self.status_indicator.style().polish(self.status_indicator)
        self.connect_button.setText("연결 해제" if connected else "AirSim 연결")
        self._set_controls_enabled(connected)
        self.message_label.setText(message)

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in (
            self.arm_button,
            self.disarm_button,
            self.takeoff_button,
            self.hover_button,
            self.land_button,
            self.emergency_button,
            self.apply_spawn_button,
            self.move_button,
            self.square_button,
            self.heart_button,
        ):
            widget.setEnabled(enabled)

    def _on_telemetry(self, data: dict) -> None:
        self._telemetry = data
        self.minimap.set_drone(float(data["x"]), float(data["y"]))
        self.telemetry_labels["landed"].setText(str(data["landed"]))
        self.telemetry_labels["position"].setText(f'{data["x"]:.2f} / {data["y"]:.2f} m')
        self.telemetry_labels["altitude"].setText(f'{data["altitude"]:.2f} m')
        self.telemetry_labels["speed"].setText(f'{data["speed"]:.2f} m/s')
        self.telemetry_labels["velocity"].setText(
            f'{data["vx"]:.2f}, {data["vy"]:.2f}, {data["vz"]:.2f} m/s'
        )
        self.telemetry_labels["attitude"].setText(
            f'{data["roll"]:.1f}° / {data["pitch"]:.1f}° / {data["yaw"]:.1f}°'
        )

    def _on_images(self, images: dict) -> None:
        self.camera_viewer.update_images(images)

    def _on_lidar(self, snapshot: object) -> None:
        points, pose = snapshot
        self.lidar_viewer.update_points(points)
        if not points.size:
            return
        valid = np.linalg.norm(points, axis=1) > 0.15
        local = points[valid]
        yaw = float(pose["yaw"])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(local)
        world[:, 0] = float(pose["x"]) + cosine * local[:, 0] - sine * local[:, 1]
        world[:, 1] = float(pose["y"]) + sine * local[:, 0] + cosine * local[:, 1]
        world[:, 2] = float(pose["z"]) + local[:, 2]
        self.planner.set_obstacle_points(world)
        self.minimap.set_obstacles(world)

        if not self.auto_replan_checkbox.isChecked() or not self._active_target:
            return
        if self._telemetry is not None:
            remaining = math.hypot(
                self._active_target[0] - float(self._telemetry["x"]),
                self._active_target[1] - float(self._telemetry["y"]),
            )
            if remaining < 1.5:
                self._active_target = None
                self._planned_path = []
                self.minimap.set_path([])
                self.message_label.setText("목표 B에 도착했습니다.")
                return
        now = time.monotonic()
        route_for_check = list(self._planned_path)
        if self._telemetry is not None:
            route_for_check.insert(
                0,
                (
                    float(self._telemetry["x"]),
                    float(self._telemetry["y"]),
                    float(self._telemetry["altitude"]),
                ),
            )
        if now - self._last_replan > 1.5 and self.planner.route_blocked(route_for_check):
            self._last_replan = now
            try:
                self._plan_and_fly(replan=True)
            except Exception as exc:
                self.worker.submit("emergency", priority=0)
                self._on_error(f"재탐색 실패 · 호버링: {exc}")

    def _on_command_completed(self, command: str) -> None:
        names = {
            "arm": "ARM/DISARM 명령 전송",
            "spawn": "선택한 스폰 A 위치를 적용했습니다.",
            "takeoff": "이륙 명령 전송",
            "hover": "호버링 명령 전송",
            "move": "목적지 이동 시작",
            "path": "패턴 미션 시작",
            "land": "착륙 명령 전송",
            "emergency": "긴급 정지 명령 전송",
        }
        self.message_label.setText(names.get(command, command))

    def _on_error(self, message: str) -> None:
        self.message_label.setText(message)
        if not message.startswith("센서:"):
            QMessageBox.warning(self, "Mission Control", message)

    def _restore_settings(self) -> None:
        self.takeoff_altitude.setValue(float(self.settings.value("takeoff_altitude", 5.0)))
        self.destination_x.setValue(float(self.settings.value("destination_x", 10.0)))
        self.destination_y.setValue(float(self.settings.value("destination_y", 0.0)))
        self.destination_altitude.setValue(float(self.settings.value("destination_altitude", 5.0)))
        self.speed.setValue(float(self.settings.value("speed", 3.0)))

    def _save_settings(self) -> None:
        self.settings.setValue("takeoff_altitude", self.takeoff_altitude.value())
        self.settings.setValue("destination_x", self.destination_x.value())
        self.settings.setValue("destination_y", self.destination_y.value())
        self.settings.setValue("destination_altitude", self.destination_altitude.value())
        self.settings.setValue("speed", self.speed.value())

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API
        self._save_settings()
        self.worker.stop()
        self.worker.wait(2500)
        event.accept()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background:#151a22; color:#e7edf5; font-size:13px; }
            QFrame { border:none; }
            QGroupBox { border:1px solid #354052; border-radius:7px; margin-top:12px; padding:12px 8px 8px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; color:#8ecbff; }
            QPushButton { background:#293445; border:1px solid #45546a; border-radius:5px; padding:8px; }
            QPushButton:hover { background:#34445a; }
            QPushButton:disabled { color:#66707e; background:#202630; }
            QPushButton#emergency { background:#8f2735; border-color:#d94c5d; font-weight:700; }
            QDoubleSpinBox { background:#0f141b; border:1px solid #3a4658; border-radius:4px; padding:5px; }
            QTabWidget::pane { border:1px solid #354052; }
            QTabBar::tab { background:#202936; padding:9px 16px; }
            QTabBar::tab:selected { background:#31547a; }
            QLabel#title { font-size:18px; font-weight:800; color:#d8edff; }
            QLabel#connected { color:#55db8a; font-weight:700; }
            QLabel#disconnected { color:#f17a84; font-weight:700; }
            QLabel#message { background:#0f141b; border:1px solid #303a49; padding:8px; color:#b7c4d5; }
            QLabel#sensorTitle { color:#8ecbff; font-weight:700; }
            """
        )


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Autonomous Drone Mission Control")
    window = MissionControlWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
