# 전체 동작 점검 — 2026-10-10

**완벽하다고 판정할 수 없다.** 아래 결과는 ROS 2 Humble + MuJoCo CPU의 가정한 물리 모델에서 얻었다. 실물과 Isaac ROS 5 GPU 실행은 검증하지 않았다.

## 수정한 문제

- 기존 춤·애교·자세의 발 위치 계산 허용 오차를 0.5 → 0.05mm로 줄이고, 몸체 동작 전체에 40ms 관절 명령 필터를 적용했다. 춤 BPM과 관절 속도 한도는 유지했다.
- 보행의 수평 발 궤적은 지지 구간과 공중 구간의 연결점에서 속도가 같도록 바꿨다. 수직 발 궤적과 점프 궤적까지 모든 미분이 연속이라는 뜻은 아니다.
- 속도 필터가 멈췄다는 이유만으로 `READY`가 되던 정지 판정을 수정했다. 기울기·필터한 자이로·관절 오차가 0.4초 안정되고, 전환 순간 네 발 접촉이 확인돼야 한다. 접촉 문턱의 80ms 이하 짧은 변동은 허용하되, 지속적인 접촉 부족은 완료를 막는다.
- 앞발 인사도 착지 힘을 96ms 확인한 뒤 몸의 무게를 되돌린다. 도중 취소하면 먼저 발을 내리고 접촉을 확인한 다음 몸을 중앙으로 옮긴다. 착지를 확인하지 못하면 무게 이동을 유지하고 FAULT로 끝난다.
- ROS 시간의 부동소수점 오차로 지지 부족 80ms 경계에서 조기 취소되는 경우를 막았다.
- 일반 기록 명령의 기본 길이가 빠른 춤·발 스텝·박수의 전체 동작을 담도록 수정했다. 각각 50 / 49 / 59초이며 사용자 지정 길이는 그대로 사용한다.

## 원본 하드웨어 참조 기립 — 현재 기본

기본 `motion.launch.py`, `sim_control.launch.py`와 ROS 제어 노드의 설정을 `control_policy_ground_startup.json`으로 바꿨다. 목표는 원본의 **20cm 서기**다. 원본 12개 서보의 I2C 주소·핀·방향·영점·보정값은 `RobotHardwares/calibration.py`에 모았고 값을 유지했다. 앞서 변경했던 이름 기반 재정렬은 삭제하고 원본의 IK 배열 슬롯 → 서보 배열 대응을 복원했다. 이름대로 재정렬하면 현재 보정값에서 발 서보가 음수로 계산되므로, 이름만으로 물리 배선을 바꿔 해석하지 않는다.

ROS 1에서는 기존 발 배치로 12cm 대기 자세를 계산하면 후방 서보가 범위를 벗어난다. 대기를 **16cm**, 기립 목표를 **20cm**로 조정했으며 4초 곡선 전체의 서보 명령이 5~175° 안에 드는지 검사했다. ROS 2는 실제 STL 기구의 IK와 서보 수치 범위를 교차 적용했다. 두 기구 모델의 치수가 달라, 바닥 휴식에서 네 발 지지를 확보하는 중간 목표는 **17.5cm**다. 최종 서기 목표는 동일한 20cm다. 180° 회전 범위와 500~2500µs 펄스 설정을 유지하고, 범위를 벗어난 일괄 명령은 거부한다.

시뮬레이션은 모든 접힌 부품을 바닥 위에 2mm 띄운 초기 배치에서 시작해, 서보 힘 0으로 자연스럽게 바닥에 안정될 때까지 계산한다. 기본 조건은 약 6.608초의 준비 계산 뒤 영상 기록을 시작한다. 몸체나 관절 위치를 다시 쓰지 않는다. 영상은 안정된 **서보 OFF·몸통 바닥 접촉 → 현재 관절각 포착 → 다리 펴기 → 기존 서기 목표**를 보여 준다. 실물 전원 OFF 자세를 측정하거나 원본에 없는 실제 초기각을 복원한 결과는 아니다.

실제 ROS 2 Humble + MuJoCo 기록 `generated/hardware_startup/ros_ground_startup.json`에서 READY를 확인했다. 기록 3초에 시작을 요청했고 16.208초에 완료됐다. OFF 모터 힘 0 Nm, OFF 몸통/바닥 거리 오차 최대 0.059mm, 최대 기울기 4.30°, 서보 명령 **8~172°**였다. 부모·자식 링크를 제외한 자체 충돌을 켰고 접촉 힘은 0 N이었다. **앞으로 약 5.7cm 밀리므로 제자리 기립은 미검증이다.** 전류 모델·질량·마찰과 CAD 축/실물 모터 축 대응은 측정값이 아니다.

기본 로컬 및 실제 ROS 기립, 로컬 질량 +15%·마찰 0.55·IMU 잡음 기립, IMU·관절·몸통 접촉 정보 손실 3개 검사, 전체 코드 **111개** 검사를 통과했다. 새로운 20cm 설정의 춤·보행·박수는 전체 물리 재검증을 하지 않았다. 다른 보관 영상은 이전 28cm 설정 기록이다. `startup.mp4`, 영상 목록 및 ZIP만 새 기립 기록으로 갱신했다.

```bash
python simulation/tools/build_hardware_reference.py
python simulation/tools/ground_startup_simulation.py
bash simulation/tools/run_ground_startup_ros.sh
python simulation/tools/make_joystick_videos.py --refresh-startup
# ROS 2의 기본: 원본 하드웨어 참조 바닥 휴식 기립
ros2 launch bigspot_sim motion.launch.py
# 과거 28cm 동작 프로파일을 명시적으로 선택하려면
ros2 launch bigspot_sim motion.launch.py policy:=/absolute/path/to/control_policy_adaptive.json
```

## 이전 바닥 휴식 시험 — 28cm 목표

이전 시작 영상은 다리를 접고 **몸통이 바닥에 닿은 서보 OFF 상태**에서 시작했다. 당시의 바닥 휴식 시험은 28cm 서기 목표였다. 아래 수치는 과거 기록이며 현재 영상과 기본 설정은 위 하드웨어 참조 기립으로 대체됐다. 기존 30개 동작 표의 시작 자세는 서보 ON의 낮은 자세였다.

실제 ROS 2 Humble 센서·관절 전송과 MuJoCo 자유 몸체 물리 계산에서 기립 완료를 확인했다. OFF 구간 모터 힘은 0 Nm, 몸통과 바닥의 최대 거리 오차는 약 0.054 mm다. 3초에 기립을 요청하고 16.208초에 READY가 됐다. 최대 roll/pitch는 3.88°였다. 부모·자식 링크 사이를 제외한 자체 충돌을 켰으며 접촉 힘은 0 N이었다. 초기 배치 이후 몸체·관절 위치를 직접 바꾸거나 들어 올리는 외력은 넣지 않았다.

**기립 중 앞으로 41.99 cm, 옆으로 0.32 cm 밀렸다. 제자리 기립 성공을 뜻하지 않는다.** 질량·마찰·모터 전류 모델 및 실물 접힌 자세의 서보 영점은 미측정이다. 관절 각도와 몸통/바닥 접촉은 시뮬레이션 센서이며, 이 기립 경로는 실물 PWM 구동기에 연결하지 않았다. 전체 전원을 끈 동안에는 제어기를 실행할 수 없으므로 영상의 OFF는 서보 힘 0인 휴식 구간이다.

로컬 기본·질량 +15%/마찰 0.55/IMU 잡음 기립과 IMU·관절·몸통 접촉 정보 손실 3개 경로도 통과했다. 전체 코드 검사 105개 통과 후 ROS 시각 변환 오류를 수정했고, 관련 시작 검사 6개를 다시 통과했다. ROS 메시지와 시계의 동일한 정수 나노초 값을 같은 방법으로 초로 변환해, 반올림 차이 때문에 유효한 IMU가 미래 데이터로 거부되는 문제를 고쳤다.

원본 기록은 `generated/ground_startup/ros_ground_startup.json`, CSV 및 센서 JSON이다. 현재 영상 목록과 ZIP에 새 영상, 바닥 휴식 안내, 버전별 캐시 주소, `startup_validation.json`을 반영했다. 현재 42개 영상과 이전 실패 참고 5개를 전부 디코딩하고 ZIP CRC를 확인했다.

```bash
python simulation/tools/ground_startup_simulation.py --legacy
bash simulation/tools/run_ground_startup_ros.sh --legacy
```

## 기존 전체 동작 검증 결과

최종 판정 파일은 `generated/quality/all_motion_validation.json`이다. 보고서 생성기는 필요한 실행 기록·동작 결과·관절 속도·전류 한도·센서 고장 후 명령 유지·전체 코드 검사를 확인하고, 누락이나 실패가 있으면 중단한다.

| 검사 | 결과 |
|---|---|
| 선택 가능한 춤·애교·자세·박수·턴 | 30종 완료 및 복귀 |
| 전후·좌우·달리기·가속·반전·시작/정지 | 8개 통과 |
| 앞발을 들던 중 인사 취소 | 착지 후 복귀 |
| 실제 ROS 기본·취소·센서 고장 경로 | 15개 통과 |
| 실제 ROS 무게 +15%, 마찰 0.55, 자이로/가속도 잡음 | 빠른 춤·빠른 몸체 춤·앞발 인사·달리기 4개 통과 |
| 코드 검사 | 100개 통과 |

정지 수정의 영향을 받는 이동·턴 10개와 인사 취소는 `audit_final`에서 재실행했다. 나머지 동작은 `audit_current` 기록을 사용한다. 보고서에 각 사례의 원본 폴더를 기록한다. 수정 전 기록은 `audit_baseline`에 남겼다.

기존 춤·애교 9종의 **관절 명령 가속도 절댓값 99백분위** 감소율 중앙값은 약 **96.3%**다. 16ms 간격 명령의 차분 지표이며, 실물 서보의 충격·저크·사람이 느끼는 자연스러움의 점수가 아니다. 빠른 스텝 춤에는 이 감소율을 적용하지 않는다.

40ms 명령 필터는 일부 동작의 자세 추종 지연을 조금 늘린다. 로컬 몸체 춤·자세·애교의 roll/pitch 추종 RMS는 약 **0.10~0.41°**, 빠른 스텝 춤은 약 **0.63°**다. 두 상대 90° 턴의 최종 오차는 로컬 재시험에서 좌 **0.06°**, 우 **1.15°**다. 이상적인 자세 quaternion을 공급했으므로 실물 IMU 정확도를 뜻하지 않는다.

달리기 정지의 마지막 1초 평균 잔여 이동 속도는 로컬 재시험에서 **2.13mm/s**다. 판정 기준은 5mm/s 미만이고 정지 관찰 시간은 4초다. 실제 정지 시간·정지 거리는 실물에서 측정해야 한다.

잡음 조건에서는 정지 자세의 관절 명령 변동이 남아 있다. 앞발 인사 중 명령 가속도 절댓값 99백분위는 약 1.88rad/s²였지만, 복귀 후 정지 구간은 약 80.79rad/s²였다. 실제 각도 변화가 작아도 차분 지표는 클 수 있다. 이번 통과는 완전한 진동 제거 또는 모든 관절의 가속도·저크 제한을 검증한 결과가 아니다.

## 재현

```bash
# 선택 목록 전체 + 이동 검사. 각 에피소드에 중력·자유 몸체·관절 모터·접촉 적용.
python simulation/tools/all_motion_audit.py --folder audit_current --include-movement
python simulation/tools/all_motion_audit.py --folder audit_final --scenarios \
  run forward backward left right accelerate smooth_reverse smooth_start_stop \
  turn_left turn_right salute_cancel

# 실제 ROS 경로. WSL Ubuntu / ROS 환경에서 실행.
bash simulation/tools/run_motion_audit_ros.sh --folder audit_ros --scenarios \
  dance_funk dance_rhythm cute_wave salute_left salute_cancel seated_clap \
  turn_left turn_right run smooth_reverse funk_cancel funk_joint_dropout \
  funk_imu_dropout funk_contact_dropout rhythm_sonar_dropout
bash simulation/tools/run_motion_audit_ros.sh --folder audit_ros --holdout \
  --scenarios dance_funk dance_rhythm cute_wave run

python -m unittest discover -s simulation/tests -v > simulation/generated/audit_unit_tests.log 2>&1
python simulation/tools/make_all_motion_report.py
```

같은 ROS 도메인에 별도 제어 노드를 중복 실행하지 않는다. 새 기본 춤은 120 BPM `dance_funk`이고, 선택 목록은 30종이다. 조이스틱 선택·실행·데드맨 경로는 코드 검사와 실제 ROS 메시지로 시험했으며 실물 게임패드 버튼 번호는 미확인이다.

## 남은 조건

- RDS5160 12개·7.4V·30A·180° 설정을 사용한다. 토크/속도/전류 곡선, 모터 감쇠, 질량, 마찰, 초음파 위치는 측정값이 아니다.
- 관절·접촉 센서는 시뮬레이션 값이다. 새 ROS 2 동작은 실물 ROS 1 PWM 구동 경로에 연결하지 않았다.
- 대부분의 자체 충돌을 끈 모델이다. 박수의 앞발 간 접촉만 별도로 켰다. 실제 다리·몸통 간섭을 검증하지 않았다.
- 점프·두발 서기·물구나무는 성공 검증을 통과하지 못한 실험 동작이며 기본 비활성이다. 30종 완료 수에 넣지 않았다.
- 센서 고장 처리의 관절 명령 유지는 전원 차단 비상 정지와 다르다.

이 결과만으로 실물에서 모든 동작이 빠르고 정확하게 된다고 말할 수 없다.
