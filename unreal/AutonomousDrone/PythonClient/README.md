# Python Mission Control

Cosys-AirSim 멀티로터를 제어하고 RGB, Depth, Segmentation, LiDAR 센서를
동시에 확인하는 데스크톱 관제 UI입니다.

## 실행

1. Unreal Editor에서 `Play`를 실행합니다.
2. `run_mission_control.bat`을 더블클릭합니다.
3. UI에서 **AirSim 연결**을 누릅니다.
4. ARM 후 이륙·이동·호버링·착륙 명령을 사용합니다.

## 미니맵 자율 이동

1. **스폰 A 선택**을 누르고 미니맵에서 시작 위치를 클릭합니다.
2. 비행 전 **선택한 위치에 스폰 적용**을 누릅니다.
3. **목표 B 선택**을 누르고 목적지를 클릭합니다.
4. 이륙한 뒤 **A* 경로로 목표 B 이동**을 누릅니다.
5. 빨간 점은 LiDAR 장애물, 파란 선은 계획된 웨이포인트 경로입니다.

경로계획기는 요청 고도를 유지하며 좌우 우회로를 탐색합니다. 이동 중 현재
경로에서 장애물이 발견되면 안전 조건을 확인한 뒤 경로를 다시 계산합니다.
스폰 이동은 AirSim 시뮬레이션
전용 기능이며 착륙 상태에서만 사용할 수 있습니다.

`settings.json`의 `Lidar1`에는 다음 항목이 필요합니다.

```json
"DataFrame": "SensorLocalFrame"
```

## 폴더

- `ui`: PySide6 관제 화면과 AirSim 작업 스레드
- `common`: AirSim API 래퍼, 좌표 변환, 안전 제한
- `missions`: 사각형·하트·배송 경로 생성
- `perception`: 카메라·LiDAR 표시와 향후 객체 탐지 확장점
- `tests`: 연결 및 비행 단위 테스트

UI의 고도는 양수이지만 AirSim에 전달할 때는 NED 좌표의 음수 Z로 자동
변환됩니다. 긴급 정지는 진행 중인 마지막 이동 명령을 취소하고 호버링을
요청합니다. 실제 비행 장비가 아닌 시뮬레이션 전용 도구입니다.
