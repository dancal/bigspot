# BigSpot Autonomy QDD

기존 서보 프로젝트를 보존하고 따로 만든 **설계·제어·자율 이동 실험 프로젝트**다. 실물 완성품은 아니다.

## 현재 연동 변경

Pi 4 R2 9.468kg 모델에 외부 제어기 → 관절 목표 → 모터 로컬 PD → 물리 시뮬레이터 경로를 구현했다. 제어·전송 시험 **24개**, CPU 물리·오류 시나리오 **11개**가 통과했다. CPU 장애물 회피는 3m 목표에 약 0.152m까지 도달했다. [실행 절차와 검증 범위](docs/isaac_integration.md), [물리 검증 결과](validation/control_loop.json)를 참고한다.

Isaac Sim 6.0.1 stereo/depth ROS bridge 및 Isaac ROS 5.0 Visual SLAM → nvblox → Nav2 연결 코드를 추가했다. **새 ROS DDS 경로와 Isaac GPU 런타임은 아직 실행하지 않았다.** 현재 PC는 RTX 5070 Laptop 8GB, Windows, 드라이버 573.22이며 [환경 검사 결과](validation/current_host.json)에 차단 조건을 기록했다. 이 변경의 기본 launch는 이전 7.48kg 모델 대신 Pi 4 모델을 사용한다.

기존 저장소: `../bigspot`, 커밋 `d8db47790a6dd9cec28b197f240ba787c3a5a504`. 기존 조이스틱·춤·서보 보정 코드는 이 커밋에 보존했다. 새 프로젝트에서 검증한 동작은 바닥에서 일어서기, 보행, 카메라 장애물 회피다. 기존 고난도 동작이 새 모터에서도 검증됐다는 뜻은 아니다.

## Raspberry Pi 4 설계 변경 — 진행 중

현재 사용자 보드는 **Raspberry Pi 4 Model B**다. 아래의 7.48kg 선정안과 자율 이동 결과는 이전 기준 모델이다. Pi 4 부품을 추가한 별도 설계는 `config/pi4_assembly.json`, `config/pi4_design.json`, `design/pi4/`에 보존했다. 이전 기준 모델을 덮어쓰지 않았다.

Pi 4 설계는 144개 유효 입체, 13개 관절 그룹, 12개 회전 관절 정의를 포함한다. CAD 질량·관성을 사용한 추정 무게는 **9.468kg**이다. 서기 높이 245mm, 발 중심 전방 45mm로 조정했다. `validation/pi4_dynamics.json`의 일어서기·보행·무게 +15%/마찰 0.55 시험 세 가지는 통과했다. 이 시험의 충돌 형상은 단순화한 물리 모델이며, 상세 CAD의 전체 간섭 검증을 뜻하지 않는다.

`design/pi4/BigSpot_Pi4_assembly.step`는 Fusion에서 열 수 있는 조립 배치 모델이다. `fusion/BigSpotPi4`에는 관절을 만드는 Fusion 스크립트가 있지만 **Fusion 내부 실행과 F3D 저장은 아직 검증하지 않았다**. 실물 제작 도면이 아니다. [현재 구성과 남은 문제](docs/pi4_design_status.md)를 먼저 확인한다.

```sh
pip install -r requirements-cad.txt
python tools/build_pi4_assembly.py
python tools/validate_pi4.py
```

Pi 4에는 Ubuntu 22.04 arm64 / ROS 2 Humble CPU 구성을 계획한다. Isaac ROS 5.0 GPU 스택은 Pi 4에서 실행할 수 없다. 실물 모터 출력용 MCU 펌웨어와 전원 보호 회로는 아직 구현·검증하지 않았다.

## R2 CAD와 동작 영상

그라파이트 통풍 커버·청록색 트림, 연결된 전자부품 트레이, 아래 12° 카메라 크래들, 접힌 다리의 여유 공간과 발 지지 구조를 수정했다. 상세 결과는 [CAD 검사](design/pi4/interference_audit.json)에 있다. 검사는 표본 자세에서 수행하며 연속 운동 전체의 보증은 아니다.

`sh
python tools/validate_control_loop.py --render
python tools/audit_pi4_cad.py --sweep --recorded
python tools/render_cad_review.py
python tools/make_motion_gallery.py
python -m http.server 59355 --bind 127.0.0.1 --directory generated/motion_videos
`

http://127.0.0.1:59355/에서 CAD 회전 영상과 동작·고장 대응 영상 11개를 재생한다. 실제 CAD 외형을 사용하는 CPU 시뮬레이션이며 1배속이다. 영상은 재생성 가능한 로컬 산출물로 Git에 포함하지 않는다.

## 이전 기준 모델의 선정안

| 항목 | 구성 | 확인 상태 |
|---|---|---|
| 좌우 벌림 | RobStride RS00 ×4 | 제조사 CAN 명령 형식 확인, 실물 미시험 |
| 엉덩이·무릎 | RobStride RS02 ×8 | 제조사 CAN 명령 형식 확인, 실물 미시험 |
| 다리 | 상·하부 각각 140mm, 발 중심 앞으로 25mm | CPU 물리 시험 |
| 서기 | 다리 높이 238mm, 발 반지름 18mm | CPU 물리 시험 |
| 몸체 | 380×190×70mm, 하부 보호판 | CAD 배치 모델 |
| 무게 | 7.48kg 추정 | 저울 측정 필요 |
| 카메라 | RealSense D435i, 전방·아래 12° | 가상 깊이 카메라 시험, 실물 미시험 |
| 컴퓨터 | Jetson Orin Nano Super 후보 | Isaac ROS 5.0 성능·발열 시험 필요 |
| 통신 | CAN 1Mbps ×2, 각 6축·200Hz | 패킷 시험, 실제 버스 미시험 |
| 전원 | 12S, 만충 50.4V 후보 | BMS·회생·방열 설계 미확정 |

모터만 $1,660: RS00 4×$125 + RS02 8×$145. 혼합 수량을 합쳐 10개 할인으로 계산하지 않았다. [RS00 판매 페이지](https://www.seeedstudio.com/Robostride-00-Actuator-p-6664.html), [RS02 판매 페이지](https://www.seeedstudio.com/Robostride-02-Actuator-p-6665.html). 배송·세금·카메라·전원·가공비는 별도다.

기존 7.4V 배터리와 PCA9685를 이 모터에 연결할 수 없다. 모터·전원·프레임·통신을 함께 바꾸는 새 설계다.

## 검증 범위

`validation/summary.json`에 실행 결과가 있다. 자유롭게 움직이는 몸체, 중력, 바닥 접촉, 모터 토크 제한을 계산했다. 초기 위치 설정 이후 몸체를 강제로 이동시키거나 외부 힘으로 들어 올리지 않았다.

- 최초 27개 후보는 무게 +15% 시험까지 통과한 구성이 없었다. 발 위치를 바꾸어 18개 후보를 추가 비교했다. 전 세계 제품과 모든 설계에서 최적임을 증명한 것은 아니다.
- 선정안을 다시 검증해 바닥에서 일어서기, 보행, 마찰 0.55·무게 +15% 시험을 통과했다.
- 깊이 카메라로 두 장애물을 측정하고 A* 경로를 만들어 3m 목표까지 이동했다.
- 이전 브랜치에는 Ubuntu/ROS 2 Humble DDS 시험 결과가 있다. 해당 경로는 내부 물리 제어기를 사용하므로 새 외부 제어기 경로의 검증을 뜻하지 않는다.
- IMU·관절 피드백 단절 시 0.15초 후 토크 차단, 카메라 단절 시 이동 정지를 시험했다. 토크 차단 이후 몸체가 바닥으로 내려가는 것은 정상 정지 자세와 구분한다.

카메라 시험의 위치 정보는 **시뮬레이터 정답**이다. 실제 VSLAM, 사람·계단·야외 이동, 장시간 발열, 낙하·충격 시험, 강화학습 정책은 검증하지 않았다. 보행만으로 직선 위치가 정확히 유지되지는 않아 위치 피드백이 필요하다. 열 모델은 정격의 60%를 연속 한도로 가정했으며 실측 온도 모델이 아니다. 모터 무부하 속도와 최대 토크를 동시에 낼 수 있다고 가정하지 않고 선형 토크–속도 한도를 사용했다. 구리 손실·배터리 지속 시간은 아직 계산하지 않았다.

## 로컬 실행

Python 3.10 이상에서:

```sh
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux: source .venv/bin/activate
pip install -r requirements.txt
python tests/test_core.py
python tools/validate_system.py --case all --render
python tools/make_report.py
python -m http.server 59354 --bind 127.0.0.1 --directory generated/viewer
```

브라우저에서 `http://127.0.0.1:59354/`를 연다. 모든 영상은 시뮬레이션 시간과 같은 1배속이다. 실행은 새 프로젝트 내부에서 한다. Windows에서는 이미 있는 이전 프로젝트의 Python 캐시도 사용한다. 독립 실행은 위 의존성을 설치하면 된다.

ROS 2 Humble 로컬 시험 환경에서는:

```sh
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv-linux
.venv-linux/bin/pip install -r requirements.txt
colcon build --symlink-install --base-paths . --packages-select bigspot_autonomy
bash tools/ros_smoke.sh
```

시각화 포함 ROS launch는 `.venv-linux`의 MuJoCo 패키지를 ROS 실행 Python에서 찾을 수 있게 설정한 후 `ros2 launch bigspot_autonomy simulation.launch.py`를 실행한다. `ros_smoke.sh`는 이 의존성을 갖춘 Python을 직접 사용한다. `controller_node`는 CAN 패킷을 `/can/dry_run_frames`에만 보낸다. 실물 CAN 출력은 구현·활성화하지 않았다.

## Fusion / CAD

`design/BigSpot_QDD_layout.step`를 Fusion의 **파일 → 열기 → 내 컴퓨터에서 열기**로 연다. 기존 모델과 별도 문서로 열린다. 자동 가져오기는 `fusion/BigSpotQDD`를 Fusion 스크립트로 등록해 실행한다. 이 스크립트의 Fusion 내부 실행은 아직 확인하지 않았다.

STEP는 치수와 모터·카메라 배치를 확인하는 입체 모델이다. 체결 구멍, 베어링, 케이블 길이, 방열판, 강도와 간섭까지 끝난 제작 도면이 아니다. `cad_validation.json`은 입체 유효성을 검증한다. `bigspot.urdf`는 관절·질량·관성·카메라 장착 위치를 제공한다. URDF의 다리 막대는 MJCF 캡슐을 원통으로 근사했다. Isaac Sim의 실제 충돌 검증은 따로 해야 한다.

CAD 재생성은 `pip install -r requirements-cad.txt` 후 `python tools/generate_design.py`로 한다. Fusion 원본의 편집 가능한 설계 이력은 STEP 가져오기만으로 복원되지 않는다.

## Isaac ROS 5.0

`isaac/run_sim.py`와 `ros/launch/isaac.launch.py`를 연결했다. `camera_source:=sim`(기본), `realsense`, `external`로 카메라 공급자를 선택한다. **GPU·실제 카메라에서 실행하지 않은 구현**이다. [상세 실행 절차](docs/isaac_integration.md)를 따른다.

Isaac ROS 5.0은 ROS 2 Lyrical을 사용한다. x86은 Ubuntu 24.04·NVIDIA Ampere 이상·VRAM 8GB 이상·드라이버 595 이상·CUDA 13.2 이상, Jetson은 JetPack 7.2 환경을 확인해야 한다. 현재 Windows/NVIDIA PC는 이 ROS 실행 환경이 구성되지 않았다. Isaac Sim 6.0.1 공식 최소 VRAM은 16GB이며 현재 8GB 장치의 작은 장면 실행은 미검증이다. [Isaac ROS 요구 사항](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html), [Isaac Sim 요구 사항](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html).

GPU 환경에서 공식 Isaac ROS 5.0 설치 절차를 완료한 뒤:

```sh
source /opt/ros/lyrical/setup.bash
python tools/isaac_preflight.py  # x86 확인용; Jetson은 JetPack 버전 별도 확인
colcon build --symlink-install --base-paths . --packages-select bigspot_autonomy
source install/setup.bash
ros2 launch bigspot_autonomy isaac.launch.py
```

필요 패키지는 `isaac_ros_cuvslam`, `nvblox_ros`, `nvblox_nav2`, `realsense2_camera`, Nav2의 controller/planner/smoother/behaviors/bt_navigator/velocity_smoother/collision_monitor/lifecycle_manager다. 이전 `isaac_ros_visual_slam` 실행 파일 이름을 그대로 사용하지 않는다.

좌표 연결은 `odom → base_link → camera_link → camera_*_optical_frame`이다. VSLAM이 `odom → base_link`를, 로봇 URDF와 카메라 드라이버가 고정 변환을 담당한다. 카메라 드라이버를 두 번 실행하지 않는다. Nav2의 명령은 `cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel → controller` 순서다. `TwistStamped`를 사용한다. 카메라 IMU는 VIO용이며 몸체 자세 제어용 IMU와 분리한다. 카메라 IMU 노이즈 수치는 초기값이라 실제 보정이 필요하다.

다음은 [실물 제작 전 확인할 항목](docs/hardware_plan.md)이다. 실물과 GPU 환경이 확보되기 전에는 “완성”, “정확한 자율주행”, “구매 확정”으로 판단할 수 없다.
