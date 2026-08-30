"""Pure-Python wrapper around the Cosys-AirSim RPC client."""

from __future__ import annotations

import socket
from typing import Iterable

import cosysairsim as airsim
import numpy as np

from common.coordinates import altitude_to_ned_z, radians_to_degrees
from common.safety import validate_destination


class AirSimController:
    def __init__(
        self,
        vehicle_name: str = "SimpleFlight",
        lidar_name: str = "Lidar1",
        host: str = "127.0.0.1",
        port: int = 41451,
    ) -> None:
        self.vehicle_name = vehicle_name
        self.lidar_name = lidar_name
        self.host = host
        self.port = port
        self.client: airsim.MultirotorClient | None = None

    @property
    def connected(self) -> bool:
        return self.client is not None

    def connect(self) -> None:
        try:
            with socket.create_connection((self.host, self.port), timeout=1.5):
                pass
        except OSError as exc:
            raise ConnectionError(
                "AirSim RPC 서버가 열려 있지 않습니다. Unreal에서 Play를 먼저 실행하세요."
            ) from exc
        client = airsim.MultirotorClient(ip=self.host, port=self.port)
        if not client.ping():
            raise ConnectionError("AirSim RPC ping에 실패했습니다.")
        self.client = client

    def disconnect(self) -> None:
        if self.client is not None:
            try:
                self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
            except Exception:
                pass
        self.client = None

    def _require_client(self) -> airsim.MultirotorClient:
        if self.client is None:
            raise ConnectionError("먼저 AirSim에 연결하세요.")
        return self.client

    def arm(self, armed: bool) -> None:
        client = self._require_client()
        client.enableApiControl(True, vehicle_name=self.vehicle_name)
        client.armDisarm(armed, vehicle_name=self.vehicle_name)

    def set_spawn(self, x_m: float, y_m: float) -> None:
        """Teleport the simulated vehicle before flight; this is not a real-aircraft API."""
        client = self._require_client()
        state = client.getMultirotorState(vehicle_name=self.vehicle_name)
        if state.landed_state != airsim.LandedState.Landed:
            raise RuntimeError("스폰 위치는 착륙 상태에서만 변경할 수 있습니다.")
        client.enableApiControl(False, vehicle_name=self.vehicle_name)
        pose = airsim.Pose(
            airsim.Vector3r(float(x_m), float(y_m), 0.0),
            state.kinematics_estimated.orientation,
        )
        client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle_name)

    def lidar_snapshot(self) -> tuple[np.ndarray, dict[str, float]]:
        """Return local LiDAR points plus the vehicle pose needed for map projection."""
        client = self._require_client()
        data = client.getLidarData(lidar_name=self.lidar_name, vehicle_name=self.vehicle_name)
        values = np.asarray(data.point_cloud, dtype=np.float32)
        if values.size < 3:
            points = np.empty((0, 3), dtype=np.float32)
        else:
            points = values[: values.size - (values.size % 3)].reshape((-1, 3))
        state = client.getMultirotorState(vehicle_name=self.vehicle_name)
        kin = state.kinematics_estimated
        roll, pitch, yaw = airsim.quaternion_to_euler_angles(kin.orientation)
        return points, {
            "x": float(kin.position.x_val),
            "y": float(kin.position.y_val),
            "z": float(kin.position.z_val),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "qx": float(kin.orientation.x_val),
            "qy": float(kin.orientation.y_val),
            "qz": float(kin.orientation.z_val),
            "qw": float(kin.orientation.w_val),
        }

    def takeoff(self, altitude_m: float) -> None:
        client = self._require_client()
        validate_destination(0.0, 0.0, altitude_m, 2.0)
        client.enableApiControl(True, vehicle_name=self.vehicle_name)
        client.armDisarm(True, vehicle_name=self.vehicle_name)
        client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        client.moveToZAsync(
            altitude_to_ned_z(altitude_m),
            velocity=2.0,
            vehicle_name=self.vehicle_name,
        )

    def hover(self) -> None:
        self._require_client().hoverAsync(vehicle_name=self.vehicle_name)

    def move_to(self, x_m: float, y_m: float, altitude_m: float, speed_mps: float) -> None:
        validate_destination(x_m, y_m, altitude_m, speed_mps)
        lookahead_m = max(5.0, float(speed_mps) * 2.0)
        self._require_client().moveToPositionAsync(
            float(x_m),
            float(y_m),
            altitude_to_ned_z(altitude_m),
            float(speed_mps),
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(False, 0),
            lookahead=lookahead_m,
            adaptive_lookahead=0,
            vehicle_name=self.vehicle_name,
        )

    def move_path(self, points: Iterable[tuple[float, float, float]], speed_mps: float) -> None:
        point_list = list(points)
        if not point_list:
            raise ValueError("비행 경로가 비어 있습니다.")
        for point in point_list:
            validate_destination(point[0], point[1], point[2], speed_mps)

        # A single destination does not need path-following control. Using
        # moveOnPathAsync for one point can repeatedly adjust path heading and
        # make the multirotor appear to oscillate near the target direction.
        # 목적지가 하나뿐이면 경로 추종 제어가 필요하지 않습니다. 단일 지점에
        # moveOnPathAsync를 사용하면 방향을 반복 보정해 기체가 떨릴 수 있습니다.
        if len(point_list) == 1:
            x_m, y_m, altitude_m = point_list[0]
            self.move_to(x_m, y_m, altitude_m, speed_mps)
            return

        path = [
            airsim.Vector3r(x, y, altitude_to_ned_z(altitude))
            for x, y, altitude in point_list
        ]
        # The planner grid is 2.5 m. AirSim's automatic lookahead was only
        # about 1.8 m at the normal mission speed, so the controller reacted
        # to nearly every grid corner. Looking several metres ahead produces
        # one continuous trajectory through the short A* segments.
        # 경로계획 격자는 2.5m이지만 기본 선행거리는 약 1.8m여서 각 격자 모서리마다
        # 제어가 반응했습니다. 선행거리를 늘려 짧은 A* 구간을 연속 경로로 추종합니다.
        lookahead_m = max(5.0, float(speed_mps) * 2.0)
        self._require_client().moveOnPathAsync(
            path,
            float(speed_mps),
            # A multirotor can translate without continuously turning toward
            # every short A* segment. This prevents heading corrections from
            # producing visible left/right jitter.
            # 멀티로터는 각 A* 구간 방향으로 계속 회전하지 않고도 이동할 수 있습니다.
            # 불필요한 방향 보정 때문에 좌우로 흔들리는 현상을 방지합니다.
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(False, 0),
            lookahead=lookahead_m,
            adaptive_lookahead=0,
            vehicle_name=self.vehicle_name,
        )

    def land(self) -> None:
        self._require_client().landAsync(vehicle_name=self.vehicle_name)

    def emergency_stop(self) -> None:
        client = self._require_client()
        client.cancelLastTask(vehicle_name=self.vehicle_name)
        client.hoverAsync(vehicle_name=self.vehicle_name)

    def telemetry(self) -> dict[str, float | str]:
        client = self._require_client()
        state = client.getMultirotorState(vehicle_name=self.vehicle_name)
        collision = client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        kin = state.kinematics_estimated
        roll, pitch, yaw = airsim.quaternion_to_euler_angles(kin.orientation)
        velocity = kin.linear_velocity
        return {
            "landed": str(state.landed_state).split(".")[-1],
            "x": float(kin.position.x_val),
            "y": float(kin.position.y_val),
            "z": float(kin.position.z_val),
            "altitude": -float(kin.position.z_val),
            "vx": float(velocity.x_val),
            "vy": float(velocity.y_val),
            "vz": float(velocity.z_val),
            "speed": float(np.linalg.norm([velocity.x_val, velocity.y_val, velocity.z_val])),
            "roll": radians_to_degrees(roll),
            "pitch": radians_to_degrees(pitch),
            "yaw": radians_to_degrees(yaw),
            "has_collided": bool(collision.has_collided),
            "collision_timestamp": float(collision.time_stamp),
            "collision_object": str(collision.object_name),
            "collision_x": float(collision.impact_point.x_val),
            "collision_y": float(collision.impact_point.y_val),
            "collision_z": float(collision.impact_point.z_val),
            "collision_normal_x": float(collision.normal.x_val),
            "collision_normal_y": float(collision.normal.y_val),
        }

    def camera_images(self) -> dict[str, bytes]:
        requests = [
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, True),
            airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, True),
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, True),
        ]
        responses = self._require_client().simGetImages(requests, vehicle_name=self.vehicle_name)
        names = ("RGB", "Depth", "Segmentation")
        return {
            name: bytes(response.image_data_uint8)
            for name, response in zip(names, responses)
            if response.image_data_uint8
        }

    def lidar_points(self) -> np.ndarray:
        points, _pose = self.lidar_snapshot()
        return points
