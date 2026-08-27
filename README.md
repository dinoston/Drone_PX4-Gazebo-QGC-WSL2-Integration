# PX4 + Gazebo + QGroundControl on Windows/WSL2

Windows 10/11에서 WSL2 Ubuntu 24.04, PX4 SITL, Gazebo Harmonic 및
QGroundControl을 구성하고 실행하는 방법을 정리한 저장소입니다.

> 이 저장소의 비행 과정은 시뮬레이션입니다. 실제 기체를 제어하지 않습니다.

## 구성

- Ubuntu 24.04 WSL 배포판을 `E:\WSL\Ubuntu-24.04`에 설치
- PX4-Autopilot을 WSL 홈 디렉터리에 설치
- Gazebo Harmonic에서 x500 멀티콥터 실행
- Windows QGroundControl과 WSL의 PX4 SITL 연결

## 1. WSL2와 Ubuntu 설치

관리자 PowerShell에서 다음 스크립트를 순서대로 실행합니다.

1. `scripts/enable-wsl2-e-drive.ps1`
2. Windows 재부팅
3. `scripts/install-latest-wsl-msi.ps1`
4. `scripts/install-ubuntu-e.ps1`

Ubuntu를 처음 실행하면 Linux 사용자명과 비밀번호를 설정합니다.

## 2. PX4와 Gazebo 설치

Ubuntu 터미널에서 실행합니다.

```bash
sudo apt update
sudo apt install -y git
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

설치 후 Windows를 재부팅합니다.

## 3. 시뮬레이션 실행

`scripts/start-px4-sim.ps1` 안의 `$LinuxUser`가 자신의 Ubuntu 사용자명과
일치하는지 확인한 다음 PowerShell에서 실행합니다.

이 스크립트는 WSLg에서 Gazebo 창이 보이지 않는 경우를 피하기 위해 X11과
소프트웨어 렌더러를 사용합니다.

PX4 콘솔에 `pxh>`가 나타나면 Windows 호스트 주소를 확인합니다.

```bash
ip route show default
```

출력의 `via` 다음 주소를 사용하여 아래 명령을 `pxh>` 콘솔에 입력합니다.

```text
mavlink start -u 14556 -o 14550 -t <WINDOWS_HOST_IP> -r 4000000 -x
```

예시:

```text
mavlink start -u 14556 -o 14550 -t 172.30.64.1 -r 4000000 -x
```

QGroundControl을 실행하면 기체가 자동으로 연결됩니다.

## 4. 간단한 비행 테스트

PX4의 `pxh>` 콘솔에서 다음 명령을 사용합니다.

```text
commander takeoff
commander land
```

비행 중 QGroundControl 지도에서 가까운 지점을 클릭하고 **여기로 이동**을
선택한 뒤 확인 슬라이더를 밀면 수평 이동을 시험할 수 있습니다.

## 5. 종료

1. 먼저 `commander land`로 착륙합니다.
2. 콘솔에 `Disarmed by landing`이 나타날 때까지 기다립니다.
3. PX4 터미널에서 `Ctrl+C`를 누릅니다.
4. Gazebo와 QGroundControl을 닫습니다.

Gazebo의 Reset 버튼은 동적으로 생성된 `x500_0` 기체를 제거할 수 있으므로
사용하지 않는 편이 안전합니다.

## 참고

- QGroundControl의 지도는 실제 지도이지만 Gazebo는 별도의 가상 세계입니다.
- 처음에는 현재 위치에서 10~100m 이내로 이동 시험을 권장합니다.
- WSL을 재시작하면 Windows 호스트 IP가 달라질 수 있습니다.
- 공식 문서: [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)

## Unreal Engine + Cosys-AirSim 프로토타입

`unreal/AutonomousDrone`에는 Unreal Engine 5.5와 Cosys-AirSim으로 구성한
드론 자율비행 프로토타입이 포함되어 있습니다.

포함된 항목:

- Unreal Engine 5.5 C++ 프로젝트와 프로젝트 설정
- `AirSimGameMode` 기본 게임 모드 설정
- 기본 테스트 레벨 및 프로젝트 콘텐츠
- Python 이륙, 5초 호버링, 착륙 연결 테스트

AirSim 플러그인은 약 2.8GB의 생성 파일과 바이너리를 포함하므로 저장소에
직접 포함하지 않습니다. 아래 공식 릴리스에서 Unreal 5.5용
`AirSim_plugin_Windows_55_33.zip`을 받아 설치합니다.

- [Cosys-AirSim v3.3 for Unreal 5.5](https://github.com/Cosys-Lab/Cosys-AirSim/releases/tag/5.5-v3.3)

압축 파일의 `AirSim` 폴더를 다음 위치에 복사합니다.

```text
unreal/AutonomousDrone/Plugins/AirSim
```

Python API를 설치합니다.

```powershell
python -m pip install cosysairsim
```

`Documents/AirSim/settings.json`에는 다음 설정이 필요합니다.

```json
{
  "SeeDocsAt": "https://cosys-lab.github.io/Cosys-AirSim/settings/",
  "SettingsVersion": 2.0,
  "SimMode": "Multirotor"
}
```

Unreal Editor에서 프로젝트를 열고 Play를 실행한 다음 아래 파일을
더블클릭하면 드론이 이륙하고 5초간 호버링한 뒤 착륙합니다.

```text
unreal/AutonomousDrone/PythonClient/tests/run_takeoff_test.bat
```

