# Pi 4 QDD: ROS 및 Isaac 연동 상태

여기서 ROS5는 **Isaac ROS 5.0**을 뜻한다. ROS 배포판은 **ROS 2 Lyrical**이다. 현재 구현은 Pi 4 SPORT CAD 질량·관성 기준 9.833952kg, 12축 RS00/RS02 모델을 사용한다.

## 검증 범위

- `validation/control_loop.json`: 외부 ControlCore 목표 → 모터 로컬 PD → 자유 부유 MuJoCo 물리 모델. 11개 시험 통과. 낮은 자세에서 일어서기, 보행, 옆 이동, 회전, 질량 +15%/마찰 0.55, IMU·관절·제어기·명령 단절, 장애물 회피, 카메라 단절이다.
- CPU 장애물 회피는 3m 목표에 약 0.152m까지 도달했고 장애물 접촉 충격량은 0이다. CPU 실험은 시뮬레이터 정답 odometry와 저해상도 raycast 깊이를 사용한다. 실제 VSLAM/Nav2 성능을 증명하지 않는다.
- 제어·전송·CAD 내보내기·영상 서버 시험 총 29개 통과. 중복·지연·미래 timestamp, 센서 오류, 시계 역행, odometry 단절, 실제 시간 watchdog 및 원자적 출력 해제를 확인한다.
- 실제 ROS DDS 실행, Isaac GPU 실행, 실물 모터 시험은 이번 환경에서 수행하지 않았다. 기존 브랜치의 ROS 결과는 새 제어기 전달 경로의 검증으로 사용할 수 없다.

## 현재 PC

`validation/current_host.json`에서 NVIDIA RTX 5070 Laptop GPU, 보고 VRAM 8151MiB, Windows, 드라이버 573.22를 확인했다. AMD라는 기존 README 설명을 수정했다.

[Isaac ROS 5 공식 환경](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)은 x86 Ubuntu 24.04, ROS 2 Lyrical, NVIDIA 드라이버 595 이상과 CUDA 13.2 이상을 요구한다. Pi 4에서 CUDA perception을 실행할 수 없다.

[Isaac Sim 6.0.1 요구 사항](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html)은 16GB VRAM을 최소 사양으로 명시한다. 8GB GPU에서 이 작은 장면이 실행될 가능성은 있지만 아직 확인하지 않았다. Windows Isaac Sim과 별도 Ubuntu ROS 머신을 DDS로 연결하는 배치도 가능하지만, 이 실행 스크립트의 기본 경로는 Ubuntu에서 두 런타임을 함께 실행하는 방식이다. 운영체제 변경과 GPU 드라이버 설치는 수행하지 않았다.

## 제어 전달 계약

`/joint_targets`는 12축 이름·위치·timestamp와 effort 배열을 가진 `sensor_msgs/JointState`다. 12개 effort는 제한된 PD 출력 진단값이며 시뮬레이터는 현재 측정 위치·속도에서 500Hz 모터 PD를 다시 계산한다. **빈 effort 배열은 목표 위치와 함께 처리하는 출력 해제 명령**이다. 이 토픽은 일반 JointState actuator 표준이 아닌 프로젝트 내부 계약이다.

CPU/Isaac plant는 중복·오래된 목표와 잘못된 축·NaN·범위 초과 값을 거부한다. 목표 또는 실제 시간 기준 갱신이 150ms 끊기면 watchdog이 래치되어 출력을 끈다. 이후 메시지로 자동 복구하지 않으며 plant 재시작이 필요하다. 제어기 센서 오류도 래치된다. 카메라·odometry·속도 명령 단절은 서기 유지 상태로 이동을 중지한다.

모터 CAN 패킷은 `/can/dry_run_frames`에서만 확인한다. 실제 모터 출력, 실측 온도·전압 피드백, MCU watchdog/전원 차단 회로가 없으므로 실물에 연결해 구동할 수 있는 상태가 아니다. 온도·전압 기본값과 추정 토크/전력/열 여유는 실물 보호 장치가 아니다.

## CPU 재현

```sh
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python tools/export_sim_model.py
python tools/validate_control_loop.py
```

Windows에서는 가상환경 Python을 `.venv/Scripts/python.exe`로 실행한다. 실제 DDS 시험은 ROS와 colcon으로 패키지를 빌드한 뒤 `python tools/ros_smoke.py`를 실행한다. 이 시험은 내부 `physics.rollout`을 사용하지 않고 CPU plant가 받은 `/joint_targets`만으로 움직인다. 실행 결과는 `generated/ros_smoke_report.json`이다.

## Ubuntu / Isaac 실행

먼저 NVIDIA 공식 Isaac ROS 5 환경 설치와 Isaac Sim 6.0.1 설치를 완료한다. 드라이버, CUDA 및 NVIDIA apt 저장소 설정은 공식 안내를 따른다. ROS 작업 공간에 이 저장소를 clone 또는 symlink하고 아래 스크립트로 로컬 패키지를 빌드한다. Ubuntu system Python 환경에 MuJoCo와 numpy도 필요하다.

```sh
source /opt/ros/lyrical/setup.bash
export BIGSPOT_ROS_WS="$HOME/bigspot_ws"
bash tools/build_ros_workspace.sh "$BIGSPOT_ROS_WS"
source "$BIGSPOT_ROS_WS/install/setup.bash"
python3 tools/isaac_preflight.py
ros2 launch bigspot_autonomy isaac.launch.py camera_source:=sim
```

다른 터미널에서 동일 ROS 환경을 source하고 Isaac Sim 제공 Python을 사용한다. `ISAAC_SIM_ROOT`는 설치 폴더다. Isaac Sim Python의 ROS `rclpy` 및 numpy import가 먼저 성공해야 한다. 서로 다른 Python ABI나 ROS 배포판을 섞지 않는다.

```sh
source /opt/ros/lyrical/setup.bash
source "$BIGSPOT_ROS_WS/install/setup.bash"
export ROS_DOMAIN_ID=0
"$ISAAC_SIM_ROOT/python.sh" isaac/run_sim.py
```

다른 터미널도 같은 `ROS_DOMAIN_ID`를 사용한다. 별도 컨테이너는 host network와 GPU 설정, DDS discovery, 시계·QoS·TF 검증이 추가로 필요하다.

시뮬레이터의 초기 folded 상태를 확인하고 IMU·관절·stereo·깊이 토픽이 지속 갱신되는지 확인한다. 제어기는 기본 disarmed다.

```sh
ros2 topic echo /controller/status
ros2 topic hz /joint_states
ros2 topic hz /camera/infra1/image_rect_raw
ros2 topic hz /visual_slam/tracking/odometry
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: true}'
```

상태가 `RISING` → `SETTLING` → `READY`로 바뀐 뒤 `/nvblox_node/static_map_slice`, Nav2 lifecycle 및 `odom -> base_link` TF를 확인하고 목표를 보낸다.

```sh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}'
ros2 service call /controller/enable std_srvs/srv/SetBool '{data: false}'
```

PhysX self-collision은 importer 설정에서 켜져 있고 충돌 형상은 해석용 proxy다. GPU에서 접촉 동작을 검증하지 않았으며 STEP 상세 강도 검증을 뜻하지 않는다. 실행 중 시뮬레이터 pose를 고정하거나 정답 위치를 navigation TF에 공급하지 않는다. `/sim/ground_truth/odom`은 진단 전용이다. 몸체 IMU 자세는 이상적인 시뮬레이션 측정이며 실제 IMU fusion 검증이 아니다. Stereo VSLAM 모드에서는 카메라 IMU를 요구하지 않는다.

`camera_source:=realsense`는 실제 D435i stereo+IMU VIO 입력을 선택한다. `external`은 외부에서 같은 RealSense 토픽·TF를 제공하는 VIO 입력이다. 두 경우 simulation clock을 쓰지 않는다. 실제 로봇에는 별도 body IMU와 joint state, MCU 펌웨어, CAN 물리 인터페이스가 필요하다.

## 제작 전에 남은 작업

Pi 4 보드와 GPU PC 사이 배치·ROS 배포판 호환성, 실제 CAN 응답과 발열, E-stop 전원 경로, 배터리 보호, 상세 CAD 간섭·체결·케이블·강도, 실제 camera 12° 장착 및 Fusion 네이티브 조립 실행은 미검증이다. [기구 설계 현황](pi4_design_status.md)을 함께 확인한다. 본 변경은 실물 제작 승인이나 완성품 인증이 아니다.

## R3 CAD 동작 영상

상세 CAD 외형과 수정된 충돌 프록시를 사용하는 영상 11개와 CAD 회전 영상 1개는 generated/motion_videos_pi4-r3/index.html에서 재생한다. 재생성 절차는 README에 있다. URDF 내 비접촉 외형은 collision에서 제외하고, 프레임·커버·모터 후면 지지·하부 어댑터 충돌 형상을 내보낸다. Isaac importer의 self collision은 활성화 설정했으며 실제 GPU 접촉 동작은 아직 검증하지 않았다.

URDF visual은 SPORT CAD에서 추출한 70개 OBJ 묶음을 사용하며 mm를 m로 변환하고 각 링크의 로컬 좌표를 보존한다. ROS 설치에는 `package://bigspot_autonomy/design/pi4/visual_meshes/` 리소스를 포함한다. 오프라인 Isaac 실행 스크립트는 실제 파일 존재를 확인하고 URI를 절대 `file://` 경로로 바꾼 임시 URDF를 가져온다. [NVIDIA URDF importer 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html)는 ROS 패키지 경로 매핑을 설명한다. 내보낸 CAD/URDF 좌표·질량과 경로 해석은 CPU 단위 시험으로 확인했으며 실제 Isaac 가져오기는 미검증이다.
