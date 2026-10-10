# 실행 증거

`summary.json`의 `checks`가 최종 판정이다. 정상 동작과 고장 주입을 구분한다.

- `fault_imu.json`·`fault_joints.json`은 정상 보행 `passed=false`, 예상 토크 차단 `expected_shutdown_passed=true`가 맞는 결과다.
- `camera_dropout.json`은 목표 도착 `passed=false`, 카메라 단절 시 이동 정지 `camera_dropout_stop_passed=true`가 맞는 결과다.
- `ros_smoke_report.json`은 실제 ROS 2 Humble 토픽으로 가상 카메라→경로→물리 보행을 연결한 결과다. 위치 정보는 시뮬레이터 정답이다.
- `design_search.json`은 45개 후보의 탐색 기록이다. 3개는 다리 도달 범위를 벗어나 중단했다. 탐색 초기의 몸체 관성·방향 유지 제어를 수정한 뒤 선정안을 최종 모델로 다시 검증했다. 이 탐색 기록을 동일 모델의 정밀 제품 비교나 전역 최적화 증명으로 쓰지 않는다.
- 동영상·NPZ·의존성은 `generated/`에 보관하고 Git에서 제외한다. 영상 해시와 길이는 `summary.json`에 있다.

실물 하중·온도·센서 정확도, Isaac ROS 5.0 GPU 실행, 제작 도면의 유효성을 증명하는 결과는 아니다.
