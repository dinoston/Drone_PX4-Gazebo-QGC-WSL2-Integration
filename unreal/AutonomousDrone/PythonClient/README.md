# Python Mission Control

Cosys-AirSim 멀티로터를 제어하고 RGB, Depth, Segmentation, LiDAR 센서를
동시에 확인하는 데스크톱 관제 UI입니다.

## 실행

1. Unreal Editor에서 `Play`를 실행합니다.
2. `run_mission_control.bat`을 더블클릭합니다.
3. UI에서 **AirSim 연결**을 누릅니다.
4. ARM 후 이륙·이동·호버링·착륙 명령을 사용합니다.

## 폴더

- `ui`: PySide6 관제 화면과 AirSim 작업 스레드
- `common`: AirSim API 래퍼, 좌표 변환, 안전 제한
- `missions`: 사각형·하트·배송 경로 생성
- `perception`: 카메라·LiDAR 표시와 향후 객체 탐지 확장점
- `tests`: 연결 및 비행 단위 테스트

UI의 고도는 양수이지만 AirSim에 전달할 때는 NED 좌표의 음수 Z로 자동
변환됩니다. 긴급 정지는 진행 중인 마지막 이동 명령을 취소하고 호버링을
요청합니다. 실제 비행 장비가 아닌 시뮬레이션 전용 도구입니다.
