# BigSpot — Fusion / Isaac ROS 5.0 작업 상태

기준일: 2026-10-10. 원본: dancal/bigspot, 커밋 `661f8de`.
작업 브랜치: `codex/fusion-isaac-ros5`.

## 현재 결과

전체 동작의 명령 끊김·발 궤적 연결·정지 판정·앞발 착지/취소 복귀를 개선했다. 최신 검증과 남은 조건은 [전체 동작 점검](QUALITY.ko.md)에 있다. 춤 템포는 유지했다. **실물·Isaac GPU·점프/수직 자세의 성공은 검증하지 않았다.**

기본 기립은 **서보 OFF·몸통 바닥 접촉 → 현재 관절각 포착 → 다리 펴기 → 원본 20cm 서기 목표**로 조정했다. 원본 서보 보정값·입력 배열 대응·180° 범위를 반영했다. ROS 1의 전원 ON 쪼그림은 16cm, ROS 2 STL 모델의 지지 확보 중간 목표는 17.5cm다. 실제 ROS 2 Humble + MuJoCo 기립을 확인했고 서보 명령은 8~172°였다. 앞으로 약 5.7cm 밀리며 실물 OFF 각도·CAD/모터 축 대응·제자리 기립은 미검증이다. 코드 검사 111개 통과. 시작 영상과 ZIP을 갱신했고 다른 보행·춤 영상은 이전 28cm 설정 기록이다.

| 항목 | 상태 |
|---|---|
| Fusion 설치·실행 | 2705.1.30 설치, 현재 실행 화면 확인. 제목은 무료 체험판 |
| Fusion 라이선스 | Autodesk 계정에 Fusion Personal 1석 등록 확인. 앱의 개인용 전환은 미확인 |
| 기구 자료 | STL 13개와 URDF 확보. `.f3d`/`.f3z` 원본은 저장소에 없음 |
| STL CAD 복원 | 13개 컴포넌트, 유효 솔리드 23개 STEP 생성. 독립 재불러오기·부피·치수 보존 검사 통과. 원본 설계 이력은 복원되지 않음 |
| Fusion 모델 표시 | 사용자가 모델 불러오기 완료를 확인. 관절 애니메이션·원본 설계 이력 재작성은 미검증 |
| 로컬 물리 시뮬레이션 | MuJoCo 3.15.0 CPU 실행. 중력·바닥 접촉·몸체 자유 이동·12개 관절 적용. 개선 crawl 60초 시험에서 전진 1.177m, 최대 기울기 3.082°, 넘어짐 없음 |
| ROS 1 오류 수정 | 기동 상태·시간 기반 일어서기, 정규화 quaternion, IMU 끊김·조이스틱 타임아웃, PID 시간 처리·자세 보정, IK→배선 다리 순서·서보 일괄 범위 검사 수정. 실물 실행 미검증 |
| ROS 2 로컬 실행 | WSL Ubuntu 22.04에 Humble 설치·실제 패키지 빌드·MuJoCo 센서/관절 메시지 실행. 새 wave 보행 24초, 관절 명령 1,497개 수신. 별도 ROS CLI로 노드·토픽 확인 |
| 자동 학습 | 초기 제어 값 30회 탐색 뒤, 발 배치·보행 방식을 수정하고 새 평가 기준으로 30회 재탐색. 새 기본 환경 및 마찰·무게 3조건 검증. 신경망 보행 정책·실물 학습은 아님 |
| Isaac ROS | **5.0.0 / ROS 2 Lyrical / Ubuntu 24.04**를 대상으로 설치·AprilTag 구성 작성 |
| Isaac Sim | **6.0.1** URDFImporter, 관절·시계·IMU·카메라 브리지 코드 작성. 웅크림 초기 자세와 학습 정책 연결. GPU 실행 미검증 |
| 댄스·애교·박수 | 기본 `dance_funk`: 120 BPM, 0.5초마다 대각선 스텝. 엉덩이 좌우·머리 쪽 좌우·끄덕임을 셔플·앞뒤 스텝·턴에 추가. 기존 춤·애교·박수도 유지 |
| 조이스틱 | 직접 버튼 + 조합 버튼으로 동작 30종 선택·실행. 버튼 8 박수, 12 빠른 스텝 춤. 실제 ROS Joy 경로 검증. 실물 버튼 인덱스 미확인 |
| 코드 검증 | 현재 코드 검사 100개 통과. 전체 선택 목록·이동·정지·앞발 착지 재검사는 QUALITY.ko.md 참조. 이전 몸체 춤 ROS 9개·빠른 춤 ROS 9개·한 발 탭 ROS 9개·큰 춤/박수 ROS 14개 기록 보존 |
| 작업 환경 | 사용자 요청에 따라 로컬 PC만 사용. AMD RX 570에서 Isaac Sim 실행 불가, NVIDIA GPU 실행 미검증 |

**STL 솔리드 복원, 로컬 ROS 물리 시뮬레이션, 기동·자세 보정·정지, 제어 값 자동 탐색까지 실행했다. 원본 스케치 재작성, 기구 형상 개선, 실물 시험, Isaac GPU 실행은 완료하지 않았다.**

## 빠른 스텝 춤 — 현재 구성

현재 `dance_funk`에는 **엉덩이 좌우·머리 쪽 좌우·끄덕임** 3구간을 추가했다. 스텝 간격 0.5초와 120 BPM은 유지한다. 계획은 8구간, 35초다. 목 관절이 없으므로 머리 쪽 움직임은 몸체와 다리로 표현한다. 현재 실행·영상 경로는 `generated/groove/`이며 [추가 동작 검증](GROOVE.ko.md)을 따른다. 버튼 5 + 12와 30종 선택 목록은 그대로다.

아래는 직전 5구간 빠른 춤의 기록이다.

기본 춤은 **`dance_funk`, 120 BPM**이다. 이전 춤의 한 발 5.67초 주기에서 **대각선 두 발을 0.5초마다 교대**하는 보행으로 바꿨다. 제자리 리듬 → 좌우 셔플 → 앞뒤 스텝 → 턴 → 혼합 각 8박자에 몸체 기울기·높이 리듬을 겹친다. [Spot 공식 안무](https://dev.bostondynamics.com/docs/concepts/choreography/move_reference.html)의 한 박자 스텝과 두 박자 몸 흔들기를 참고했다.

실제 ROS 주 안무 20초에서 대각선 쌍 스텝 39회, 간격 중앙값 0.500초를 측정했다. 셔플 좌우 이동 폭 **7.9 cm**, 앞뒤 이동 폭 **12.9 cm**, 턴 yaw 변화 폭 **39.4°**, 발 최대 높이 **26~35 mm**다. 계획 23초, 안정 확인 후 약 23.4초에 완료했다. **9개 실제 ROS 시험과 91개 코드 검사**를 통과했다. 센서 중단·동적 지지·전류 모델의 조건은 [빠른 춤 검증](FUNK.ko.md)에 있다.

**버튼 5 유지 + 12 빠른 춤 / 8 박수 / 13 애교**, 1 정지. 선택 목록은 30종이다. `generated/funk/dance_funk.mp4`는 직접 합성한 120 BPM 소리를 포함한 25초 정상 속도 영상이며, `funk_preview.gif`는 16초 발췌다. 실제 몸체·관절 기록을 사용했다. **ROS 2 Humble + MuJoCo CPU 결과이며 실물·Isaac GPU 실행은 미검증**이다.

## 한 발 탭 춤 — 이전 기록

아래 버튼·기본값·목록 개수는 이 단계 당시 기록이다. 현재는 위 빠른 춤과 30종 목록을 사용한다.

기본 춤은 `dance_steps`다. **오른쪽 앞 → 왼쪽 뒤 → 왼쪽 앞 → 오른쪽 뒤** 순서로 체중을 옮겨 한 발씩 들고, 앞·뒤와 바깥쪽으로 탭한 뒤 착지한다. 물리 기록에서 앞발은 약 41 mm, 뒷발은 약 37 mm 들고 전후로 35~45 mm 움직였다. 네 번의 착지를 확인했고 기본·질량/마찰/잡음 조건 모두 최소 세 발 지지를 유지했다. 마무리는 약 49 mm 몸체 낮추기와 기립이다.

안무 90 BPM, 한 발의 전체 체중 이동·탭·착지·복귀 주기는 약 5.67초다. 전체 경로는 32초다. 다른 세 발 지지가 확인될 때 발을 들고, 착지 힘이 96 ms 연속 확인될 때 다음 체중 이동으로 진행한다. **실제 ROS 시험 9개**에서 완료·공중 발 중지 복귀·센서 단절 명령 유지까지 검증했다. 자세한 측정과 실행 명령은 [발 스텝 춤 검증](STEPS.ko.md)에 있다.

**버튼 5 유지 + 12 발 스텝 춤 / 8 박수 / 13 애교**, 1 정지다. 선택 목록은 29종이며 기존 몸체 춤을 선택할 수 있다. `generated/steps/dance_steps.mp4`는 33.5초, `steps_preview.gif`는 24초 정상 속도 기록이다. 앞·뒤 고정 카메라로 실제 물리 상태를 보여 준다. **ROS 2 Humble + MuJoCo CPU 결과이며 Isaac ROS 5.0·실물 실행 결과는 아니다.**

## 큰 춤·앉아서 박수 — 이전 기록

이 단계의 기본 춤은 `dance_show`였다. 직전 빠른 춤 대비 높이 변화 폭 **29.3 → 69.9 mm(2.38배)**, roll 변화 폭 **5.9 → 17.5°(2.97배)**로 확대했다. 90 BPM, 몸체 주기 4초다. 기존 `dance_rhythm`의 2.67초 주기보다 길게 잡아 큰 동작에서 발 지지를 확보했다. 두 춤 모두 선택할 수 있다.

`seated_clap`은 뒷다리 아래쪽 접촉면으로 지지하고, 앞발을 들어 **실제 박수 세 번 → 앞발 착지 확인 → 웅크리기 → 기립**을 완료한다. 몸통은 바닥에 닿지 않는다. 기본 조건과 질량 +15%·마찰 0.55·자이로/가속도 잡음 조건 모두 완료했다. 새 실제 ROS 시험 14개 및 코드 검사 77개를 통과했다.

이 단계의 버튼 12는 큰 춤, 선택 목록은 28종이었다. 현재 버튼 12는 빠른 스텝 춤이며 목록은 30종이다. 실패했던 수직 자세 실험은 버튼 8에서 제외했다. 실제 게임패드 버튼 번호·실물 구동은 미검증이다. 자세한 기준, 센서 중단 결과, 실행 명령은 [큰 춤·박수 검증 기록](SHOW.ko.md)에 있다.

`generated/show/dance_show.mp4`는 23초, `seated_clap.mp4`는 45초 정상 속도 영상이다. 고정 카메라를 사용했다. 박수 영상은 실제 ROS 물리 상태 기록을 앞쪽 시점에서 다시 렌더링했으며 몸체·관절 위치를 바꾸지 않았다. **ROS 2 Humble + MuJoCo CPU 결과이며 Isaac ROS 5.0·실물 실행 결과는 아니다.**

## 빠른 템포·조이스틱 선택 — 이전 기록

아래 버튼 구성은 이전 기록이다. 현재 구성은 위 표와 `config/joystick.yaml`을 따른다.

새 기본 춤은 `dance_rhythm`, **90 BPM**이다. 몸체 반복 주기는 큰 춤의 8초에서 **2.67초로 3배 빨라졌다**. 시작·끝 각 3초를 포함한 경로는 **38초 → 16.67초**다. 가속·감속 경로와 기존 관절 명령 속도 한도 1.2 rad/s를 유지한다. 큰 좌우 몸체 이동을 제거하고 높이 변화·roll/pitch 리듬으로 구성해 빠른 동작에서 발 지지를 확보했다. 높이 변화 폭은 **72.9 mm → 29.3 mm**, roll 변화 폭은 **13.1° → 5.9°**다. 큰 움직임을 원하면 `dance_big`를 선택할 수 있다.

기본 조건과 질량 +15%·마찰 0.55·자이로/가속도 잡음 조건의 실제 ROS 기록 모두 넘어짐·FAULT 없이 완료했고, 접촉 힘 >1 N인 지지 발 최소 3개를 유지했다. 기본 조건의 roll/pitch 목표 추종 RMS는 약 0.22°다. 실제 질량·전류·기구 간섭은 측정하지 않았고 자체 충돌은 꺼져 있다. 이상적인 자세 quaternion을 사용했으므로 이 결과는 실물 IMU 정확도나 실물 안전 검증이 아니다. 새 춤의 다른 BPM은 실험 활성화가 필요하다. 기존 작은 춤·큰 춤의 60 BPM 초과 제한은 유지했다.

**새 실제 ROS 시험 12개**: 빠른 춤 기본·별도 조건 2개, 중간 취소·정지 버튼·데드맨 해제·Joy 메시지 단절·초음파 단절 5개, IMU·관절·접촉 단절 3개, 조합 버튼 선택 실행·발을 들던 중 데드맨 해제 2개. 선택 실행은 앞발 인사 `cute_wave`가 한 번만 요청·완료됐고, 발을 든 중지는 착지 후 복귀했다. IMU·관절·접촉 단절은 각각 약 0.26·0.24·0.10초 안에 FAULT가 됐고 이후 관절 명령 변동은 0이었다. 새 관절 명령 기록의 최대 속도는 기존 1.2 rad/s 이하다. 이는 마지막 각도 명령 유지이며 모터 전원을 차단하는 비상 정지는 아니다.

버튼 번호는 **일반 ROS Joy 인덱스**다. 동작 요청에는 버튼 5를 유지해야 한다.

| 입력 | 동작 |
|---|---|
| 스틱 축 1 / 0 / 3 | 전후 / 좌우 / 회전 |
| 버튼 0 / 1 | 일어서기 요청 / 정지 |
| 버튼 3 / 4 | 달리기 전환 / 누르는 동안 가속 |
| 버튼 6 / 7 | 인사 / 웅크렸다 복귀 |
| 버튼 10 / 11 | 좌 / 우 상대 90° 턴 |
| 버튼 12 / 13 | 90 BPM 빠른 춤 / 애교 조합 |
| 4 + 12 / 4 + 13 | 선택 목록 이전 / 다음. 선택만으로 움직이지 않음 |
| 4 + 6 | 선택한 동작 한 번 실행 |
| 5 해제 또는 Joy 단절 | 진행 중 춤·애교·선택 자세 복귀 요청 |

선택 목록은 춤 5종, 애교 4종, 몸체 자세 15종, 턴 2종으로 **26종**이다. 낮게/높게 서기·앉기·좌우 기대기·위아래 보기·좌우 기울기·기지개·몸체 원·좌우 앞발 인사도 포함한다. 버튼 2의 방향 점프와 8/9의 수직 자세 실험은 기본 비활성이고 26종 목록에는 넣지 않았다. 앞서 점프·수직 자세는 성공 기준에 미달했다.

`config/joystick.yaml`에서 축·버튼 번호를 바꿀 수 있다. `dance_action`, `cute_action`, `selection_actions` ROS 파라미터로 연결 동작·선택 순서도 지정할 수 있다. 조합 버튼을 누른 동안 스틱 속도 명령은 0이며, 버튼을 유지해도 요청은 한 번만 전송한다. `/bigspot/joystick_state`는 현재 선택 이름·순서·목록을 발행한다. 실제 PS4 버튼 이름과 번호는 미확인이다. **현재 WSL `/dev/input` 장치는 없으므로 실제 게임패드 입력은 검증하지 못했다. ROS 1 실물 PWM 경로에 새 춤/선택 목록은 연결하지 않았다.**

```bash
# 로컬 CPU: 90 BPM 정상·별도 조건·선택 버튼 재현
bash simulation/tools/run_tempo_ros_demo.sh --group main
bash simulation/tools/run_tempo_ros_demo.sh --group checks
# 별도 시뮬레이터가 센서·관절 정보를 제공하는 환경
ros2 launch bigspot_sim motion.launch.py rhythm_bpm:=90.0
ros2 topic echo /bigspot/joystick_state
# 직접 동작 요청
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: dance_rhythm}'
```

기록 도구가 제어기를 실행하므로 같은 ROS 도메인에 launch 제어기를 중복 실행하지 않는다. `generated/tempo/rhythm_demo.mp4`는 정상 속도 18.5초 영상, `rhythm_preview.gif`는 10초 발췌다. `tempo_validation.json/png`와 12개 ROS CSV·센서 기록을 함께 저장했다. 재생 속도 배율은 1이다. **ROS 2 Humble + MuJoCo CPU** 결과이며 Isaac ROS 5.0 GPU·실물 실행 결과는 아니다.

## 큰 춤·애교 추가 — 이전 기록

이전 `dance`는 몸체 높이 변화 13.1 mm, roll 변화 폭 3.4°였다. `dance_big`는 실제 ROS 2 Humble / MuJoCo CPU 기록에서 **높이 변화 72.9 mm(5.6배), roll 변화 폭 13.1°(3.8배)**로 커졌다. 기본 60 BPM, 32박자, 시작·종료 각 3초로 경로는 38초다. 구간 길이를 늘려 발 지지를 유지했고, 자세가 안정된 뒤 READY로 복귀했다. 몸체 roll/pitch 목표 대비 RMS 오차는 약 0.25°, 접촉 힘 >1 N 기준 지지 발은 최소 3개였다. 재생 속도를 바꾸거나 몸체를 강제로 이동시키지 않았다.

| 이름 | 동작 | 기본 모델 실제 ROS 결과 |
|---|---|---|
| `dance_big` | 넓은 좌우 흔들기·깊게 낮췄다 일어나기·몸체 원·큰 끄덕임 | 완료, 몸체 높이 변화 약 73 mm |
| `cute_tilt` | 몸을 낮추고 좌우로 갸웃하기 | 완료, roll 변화 폭 약 11.3° |
| `cute_wave` | 몸무게 이동 → 오른쪽 앞발 들어 흔들기 → 착지 → 복귀 | 완료, 발 최대 높이 약 81 mm, 세 발 지지를 유지한 발 들기 구간 약 5.46초 |
| `cute_bow` | 앞을 숙이고 낮춰 놀자 자세·좌우 흔들기 | 완료, 높이 변화 약 43 mm, pitch 변화 폭 약 9.9° |
| `cute` | 갸웃 → 앞발 인사 → 놀자 자세 | 완료, 경로 31.5초 |

목 관절이 없으므로 갸웃·끄덕임은 **몸체 기울기**다. 큰 춤의 초기 경로는 깊은 웅크림과 뒤로 기울이기를 함께 적용한 구간에서 뒷다리 관절 범위를 벗어나 거부됐다. 낮은 자세에서 기울이는 방향·폭과 높이 조합을 바꿨다. 빠른 초기 경로는 순간 발 들림이 생겨 구간을 늘렸고, 기본 모델에서 지지 발 최소 3개를 확인한 경로를 사용한다. 관절 한도를 늘려 통과시키지 않았다.

앞발은 계획 지지 삼각형 여유 최소 12 mm와 남은 세 발의 실제 접촉 힘을 확인한 뒤 들어 올린다. 무게 이동은 발 들기 전에 끝내고, 정상 실행에서는 발을 내린 뒤 몸을 복귀한다. 앞발을 든 상태의 중지는 무게를 옮긴 자세를 유지하면서 먼저 발을 내린다. 해당 발 접촉을 확인한 뒤 몸을 복귀하며, 2.5초 안에 착지를 확인하지 못하면 그 자세의 관절 명령에서 FAULT로 정지한다. 관절 명령 속도는 기존 1.2 rad/s 한도를 유지한다.

새 실제 ROS 기록은 **14개**다. 기본 동작 5종, 큰 춤 취소·앞발 취소·초음파 단절·버튼 입력 4종, 발을 들던 중 IMU·관절·접촉 메시지 단절 3종, 질량 +15% / 마찰 0.55 / 자이로·가속도 잡음 조건 2종을 수행했다. 앞발 인사는 별도 조건에서도 통과했다. **큰 춤은 별도 조건에서 넘어짐 없이 완료했지만 순간 지지 발이 2개로 줄어 엄격한 지지 기준에 미달했다.** 기본 모델 통과를 모든 무게·바닥·센서 조건에서의 성공으로 해석하지 않는다. 실제 질량·서보 영점·모터 전류·기구 간섭·자체 충돌·실물 관절/접촉 피드백은 여전히 미검증이다. 자세 quaternion은 물리 엔진의 이상적인 값이다.

이 기록 당시 버튼 12는 `dance_big`였다. 현재 기본값은 **12 → `dance_rhythm`**, **13 → `cute`**다. 인덱스 5를 유지해야 요청하며, 해제·입력 단절·정지 버튼은 복귀를 요청한다. `dance_action`, `cute_action`으로 동작도 다시 지정할 수 있다. 실제 PS4 버튼 인덱스는 확인하지 않았다. 큰 춤과 기존 작은 춤도 조합 버튼 목록에서 선택하거나 토픽으로 요청할 수 있다.

```bash
# 이 PC의 ROS + CPU 물리 기록. 같은 도메인에 제어기를 중복 실행하지 않는다.
bash simulation/tools/run_dance_ros_demo.sh
bash simulation/tools/run_dance_ros_demo.sh --scenario cute --seconds 46 --case my_cute

# 다른 시뮬레이터에서 일어서기·센서 조건 충족 후 한 동작씩 요청
ros2 launch bigspot_sim motion.launch.py dance_bpm:=60.0 dance_button:=12 cute_button:=13
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: dance_big}'
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: cute_wave}'
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: stop}'

export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=43 MUJOCO_GL=osmesa
source /opt/ros/humble/setup.bash
source simulation/ros2_ws/install/setup.bash
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/expressive_suite.py
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/expressive_suite.py --group checks
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/expressive_suite.py --group holdout
python simulation/tools/make_expressive_report.py
```

`generated/expressive/`에 `dance_big.mp4`(42초), `cute_demo.mp4`(35.5초), `expressive_demo.mp4`(77.5초), `expressive_preview.gif`(19초), `expressive_validation.json`과 14개 기록을 보관한다. 원래 속도의 기록을 잘라 연결했고 소리는 없다. **ROS 2 Humble + MuJoCo CPU** 실행이며 Isaac ROS 5.0 GPU 실행·실물 실행·새 신경망 학습 결과가 아니다.

## 기존 댄스 3종 — 기본 60 BPM

댄스는 새 ROS 2 제어기의 `/bigspot/action`에 추가했다. 네 발의 지면 목표를 유지하며 몸체를 움직인다. 발 들기·점프는 포함하지 않는다. 음악을 듣고 박자를 추정하는 기능은 없고 지정한 BPM에 따라 안무를 실행한다. 기본 60 BPM에서는 2초 시작, 16박자 안무, 2초 종료 경로를 거쳐 자세가 안정되면 완료한다. 총 경로는 20초, 실제 센서 안정 확인을 포함한 실행 시간은 약 20.4초였다.

| 요청 이름 | 안무 | 실제 ROS 60 BPM 결과 |
|---|---|---|
| `dance_sway` | 몸체 좌우 이동·기울이기 | 완료, 지지 발 최소 4개 |
| `dance_bounce` | 몸체 낮추기·높이기와 고개 끄덕이기 | 완료, 지지 발 최소 3개, 점프 아님 |
| `dance` | 좌우 흔들기 → 상하·끄덕임 → 혼합 움직임 → 작은 몸체 원 | 완료, 지지 발 최소 3개 |

조합 안무에서 몸체 roll/pitch 목표 대비 실제 물리 자세의 RMS 오차는 **0.19°**, 몸체 높이 변화 폭은 약 **13 mm**였다. 넘어짐 없이 READY로 복귀했다. 이 수치는 실제 MPU6050 정확도가 아니다. 기본 시험은 물리 엔진의 이상적인 자세 quaternion을 ROS에 전달한다. 실제 관절·접촉도 물리 엔진 값이며 몸체 위치를 강제로 바꾸거나 재생 속도를 올리지 않았다. 몸체의 잔여 위치·yaw 변화가 있고 절대 위치 고정 기능은 없다.

시작 전 정지·정상 자세·네 발 접촉·관절 피드백·전방 초음파 여유를 요구한다. 관절 범위와 계획 속도 여유, 지지 다각형을 검사한다. 전체 경로의 무거운 역기구학 검사는 ROS 센서 타이머 시작 전에 끝내고, 요청 때 현재 자세에서 이어지는 2초 진입 경로를 별도로 확인한다. 요청 콜백이 오래 걸려 센서 갱신을 막던 오류를 수정했다. 동작 중 관절 명령 속도는 1.2 rad/s 이하로 제한한다.

정지 버튼·데드맨 버튼 해제는 현재 자세에서 부드럽게 복귀한다. 게임패드 입력이 0.3초 이상 끊기는 경우에도 조이스틱 경로가 `stop`을 요청한다. 버튼을 길게 눌러도 시작 요청은 한 번만 보낸다. 초음파가 끊기거나 전방 여유가 부족하면 복귀한다. IMU·관절·접촉 정보 자체가 사라지면 FAULT에서 마지막 관절 명령을 유지한다. 이 정지는 전원을 끄는 비상 정지가 아니다.

실제 ROS 시나리오는 **11개**다. 정상 안무 3개, 중간 취소·초음파 단절·중지 버튼·데드맨 해제 4개, IMU·접촉·관절 정보 단절 3개는 각각 의도한 결과를 확인했다. 단절 후 마지막 명령 유지와 실제 제어 타임스탬프 기준 관절 명령 속도도 기록에서 검사했다. 90 BPM·질량 +15%·마찰 0.55·IMU 각속도/가속도 잡음 시험은 끝까지 수행했지만 순간 지지 발이 2개로 줄어 **성공 기준을 통과하지 못했다**. 빠른 박자는 움직임 폭을 줄여도 이 조건에서 미달했다. 60 BPM 초과는 기본 제어기에서 거부하고 시뮬레이션 실험을 명시적으로 활성화했을 때만 허용한다. 40~60 BPM은 설정 범위이며 실제 물리 시험으로 검증한 박자는 60 BPM이다.

```bash
# 현재 PC의 실제 ROS + CPU 물리 기록. 다른 제어기를 같은 도메인에 중복 실행하지 않음
bash simulation/tools/run_dance_ros_demo.sh
# 동작 선택: --scenario dance_sway 또는 --scenario dance_bounce

# 센서 토픽을 제공하는 별도 시뮬레이터와 연결
ros2 launch bigspot_sim motion.launch.py dance_bpm:=60.0 dance_button:=12
# 먼저 일어서기·정지·센서 조건 충족 후 한 번 요청
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: dance}'
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: stop}'

# 검증 재현
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=43 MUJOCO_GL=osmesa
source /opt/ros/humble/setup.bash
source simulation/ros2_ws/install/setup.bash
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/dance_suite.py --group routines
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/dance_suite.py --group checks
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/dance_suite.py --group holdout
python simulation/tools/make_dance_report.py
```

기본 댄스 버튼은 ROS Joy 인덱스 12이며 현재 `dance_rhythm`을 요청한다. 요청 허용 버튼은 인덱스 5다. 실제 PS4 버튼 이름·인덱스는 확인하지 않았으며 `dance_button`으로 다시 지정해야 한다. ROS 1 실물 PWM 경로에는 댄스를 연결하지 않았다. 현재 실물에는 필요한 실제 관절·발 접촉 피드백이 확인되지 않아 새 댄스 시작 조건을 충족한다고 볼 수 없다. 실물 기구 간섭·질량·모터 전류·자체 충돌 검증도 필요하다.

결과는 `generated/dance/`의 `dance_full.mp4`(조합 안무 24초), `dance_demo.mp4`(3종 40초), `dance_preview.gif`(16초), `dance_validation.json`, 그래프와 11개 ROS 기록에 저장한다. 영상에는 소리가 없다. **ROS 2 Humble + MuJoCo CPU** 결과이며 Isaac ROS 5.0 GPU 실행이나 새 신경망 학습 결과가 아니다.

## 부드러운 이동·센서 조합·새 자세 13종

새 기본 `motion.launch.py`는 `control_policy_adaptive.json`을 사용한다. 아래의 기존 motion 시험은 이전 정책의 기록이다. 새 결과는 `generated/adaptive/`에 별도로 보관한다. 물리 시뮬레이션은 실제 ROS 2 Humble 메시지로 실행했고, 실물·Isaac ROS 5 GPU 실행은 하지 않았다.

속도 명령은 갑자기 바뀌어도 시작·가속·감속·정지·반대 방향 전환 때 속도와 가속도가 이어지도록 처리한다. 선형 가속도는 0.20 m/s², 가속도 변화율은 1.2 m/s³, yaw 가속도는 0.6 rad/s², yaw 가속도 변화율은 3.0 rad/s³로 제한한다. 달리기 전환 때 보행 주기·발 높이·지지 시간도 서서히 바꾼다. 상대 90° 회전 완료 때 속도를 바로 0으로 만들던 부분을 제거했다. 목표 관절 명령의 변화 제한이며, 실물 모터의 가속도나 바닥에서의 미끄러짐까지 보장하지 않는다.

| 입력 | 제어에 쓰는 방식 | 실물 확인 |
|---|---|---|
| MPU6050 자세·자이로·가속도 | 원하는 자세와의 오차·각속도를 보정. 기울기와 지속 가속이 커지면 감속. 직선 이동 시 상대 yaw 유지 | 코드 수정, 실물 영점·장착 방향 미검증 |
| 좌·우 초음파 | 가까운 값 즉시 반영, 먼 값의 이상치는 3개 중앙값으로 억제. 지연·감속·가속도 변화 시간을 고려해 전방 속도 제한 | 취득·ROS 연결 수정, GPIO 실물 시험 미실행 |
| 실제 관절 각도 | 목표 각도 추종 지연 시 감속 또는 복귀. 새 정책에서는 값이 끊기면 이동 정지 | 현재 시뮬레이션 값. PCA9685 PWM 경로에는 엔코더 피드백이 없음 |
| 발별 접촉 힘 | 앞발 들기 전 네 발 접촉 확인. 무게 이동 후 남은 세 발 지지를 확인. 지지 손실 시 복귀·감속 | 현재 시뮬레이션 값. 실제 발 접촉 센서 미확인 |
| 배터리 7.4V / 최대 30A | 물리 모델의 전류 제한과 모터 성능 근사에 반영 | 사용자가 제공한 전원 조건. 전류를 실시간 측정하는 센서는 미확인 |

초음파는 **전방만** 보호한다. 뒤·옆의 장애물, 계단·낭떠러지, 부드러운 재질·비스듬한 표면의 반사 실패를 모두 알아낼 수 없다. GPIO 측정 타임아웃을 빈 공간으로 처리하지 않고 NaN으로 전달해 전진을 멈춘다. 좌·우 어느 쪽이든 0.35초 넘게 오래된 값이면 전진을 제한한다. 기본 감속 구간은 0.60 m, 확보할 센서 앞 여유 거리는 0.22 m다. 후진은 전방 장애물에서 빠져나올 수 있지만 후방이 안전하다고 판정한 것은 아니다.

ROS `Range` 메시지의 `range` 필드를 **m** 단위로 쓰도록 기존 코드를 고쳤다. 기존 코드의 Joy/Range 타입 불일치와 `distance` 필드 오류, LED 색만 변경하던 처리를 수정했다. ROS 1 실물 실행 파일에도 거리 구독과 전방 감속·정지를 연결하고 센서 노드를 활성화했다. 두 초음파는 동시에 울리지 않고 최소 60 ms 간격으로 번갈아 읽는다. 한 센서는 약 7~8 Hz로 갱신한다. Adafruit API의 거리 단위·타임아웃은 [공식 문서](https://docs.circuitpython.org/projects/hcsr04/en/latest/api.html), ROS 필드·유효 범위는 [공식 Range 정의](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg)를 따른다. 기존 배선 D16/D19, D20/D21은 유지했으며 실물 배선 확인은 하지 않았다.

| 새 동작 이름 | 자세 |
|---|---|
| `low`, `high` | 몸체 낮추기·높이기 |
| `sit` | 네 발을 유지하며 낮게 앉는 자세, 뒤쪽으로 약간 이동 |
| `lean_left`, `lean_right` | 몸체 좌·우 이동 |
| `look_up`, `look_down` | 앞쪽을 들거나 숙이기 |
| `roll_left`, `roll_right` | 몸체 좌·우 기울이기 |
| `stretch` | 앞쪽을 숙이고 몸체를 낮추기 |
| `body_circle` | 작은 원을 따라 몸체 이동 |
| `salute_right`, `salute_left` | 지지 삼각형 안으로 무게를 옮긴 뒤 앞발 한쪽 들기 |

모든 자세는 정지·정상 자세에서 시작한다. 실행 전에 경로 전체 81개 지점의 역기구학을 검사한다. 앞발 들기는 계획상 지지 삼각형 여유 거리 최소 12 mm를 확인하고 접촉·관절 피드백을 요구한다. 각 자세는 완만한 시작·정지 경로를 거쳐 복귀하며, 기울기·각속도·관절 추종 오차가 안정될 때 완료로 판정한다. 중간 `stop`은 현재 자세에서 약 1.8초 경로로 발을 낮추고 정상 자세로 복귀한다. IMU나 필요한 접촉·관절 값 자체가 사라지면 마지막 관절 명령을 유지하는 FAULT로 들어간다.

실제 ROS 시험에서 13종 모두 수행·복귀했고, 몸체 roll/pitch 목표 대비 RMS 오차는 약 **0.18°**, 최대 오차는 약 **0.69°**였다. 대부분 시험은 물리 엔진이 제공하는 이상적인 자세 quaternion을 사용한다. 별도 시험에서는 이 필드를 미제공으로 표시하고 자이로·가속도만으로 자세를 추정했다. 그 오차는 약 **1.4° RMS**였다. 두 숫자는 센서 조건이 다르며 실물 정확도 수치가 아니다. 자이로/가속도만으로 절대 위치·이동 거리·절대 yaw를 알아낸다고 가정하지 않는다.

기본 16개 ROS 시나리오와 마찰 0.55·무게 15% 증가·IMU 잡음 조건 3개를 실행했다. 초음파 단절·NaN, 관절 피드백 단절은 감속 정지했고, IMU 단절은 예상한 `IMU timeout` FAULT로 전환했다. 옆으로 20 N 힘을 0.15초 적용한 실제 물리 외란 시험도 수행했다. 외란은 `balance_push` 시험에만 적용하며 자세 영상이나 점프를 만들기 위한 외부 힘은 사용하지 않는다. 벽 접근 시험에서 접촉 0회, 마지막 초음파 여유 거리는 약 27 cm였다. 전진 명령을 계속 보낸 상태에서 약 15.76초에 READY로 정지했다.

초음파 시뮬레이션은 좌·우 각각 5개 광선으로 장애물 표면까지 거리를 읽는 근사다. 장착 위치 `[0.24, ±0.055, 0.04] m`, yaw ±0.18 rad, 관측각 0.26 rad는 가정이다. 음향 반사·간섭·초음파 장착 오차는 모델링하지 않았다. 로봇 질량·모터 토크/전류 곡선·마찰도 실측값이 아니고 자체 충돌은 비활성이다. 세 발 자세와 기존 점프·수직 실험을 실물에 옮기려면 관절 피드백·발 접촉·실제 기구 간섭·센서 장착 보정이 먼저 필요하다. 기존 점프·물구나무 실패 상태는 그대로 유지한다.

```bash
# 현재 로컬 WSL 환경: 실제 ROS로 새 정책과 물리 센서를 함께 검증
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=43 MUJOCO_GL=osmesa
source /opt/ros/humble/setup.bash
source simulation/ros2_ws/install/setup.bash
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/adaptive_suite.py --group safety
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/adaptive_suite.py --group poses
"$HOME/.cache/bigspot-sim/venv/bin/python" simulation/tools/adaptive_suite.py --group holdout

# 개별 동작. 새로운 정책은 앞의 두 Range 토픽과 실제 JointState 입력이 필요
bash simulation/tools/run_motion_ros_demo.sh --profile adaptive --scenario look_up --seconds 20 --output-folder adaptive --case my_look_up

# 다른 시뮬레이터가 clock/IMU/Range/관절/접촉 토픽을 제공할 때 실행
ros2 launch bigspot_sim motion.launch.py
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: look_up}'
# 위 요청은 먼저 일어서기·정지·센서 유효성 조건을 만족해야 수락

# Raspberry Pi에서만 실제 GPIO 드라이버 실행. 이 데스크톱에서는 실행하지 않음
# adafruit-circuitpython-hcsr04 / Blinka와 실제 배선이 필요
ros2 run bigspot_sim ultrasonic
```

제어기는 `/bigspot/ultrasonic/left`, `/bigspot/ultrasonic/right` (`sensor_msgs/Range`)를 받는다. `/bigspot/state`에는 센서 유효성, 감속 이유, 요청·제한 속도, 현재 속도·가속도·가속도 변화율, 목표 자세를 JSON으로 제공한다. 기록에서 메시지 도착 시간과 실제 제어 시각을 구분했고, ROS 시계의 나노초 반올림과 전달 동기화를 수정해 제어 주기 경계에서 샘플이 빠지는 문제를 줄였다. `adaptive_validation.json`과 그래프는 실제 제어 타임스탬프로 가속도 변화를 다시 검증한다.

영상 `poses_showcase.mp4`는 13개 자세, `adaptive_demo.mp4`는 출발·정지·벽 감속·선택 자세·외란 회복을 보여준다. 기록 구간만 잘랐으며 재생 속도를 바꾸지 않았다.

장애물 여유 거리가 0.25 m 이하로 들어오면 정지를 유지하고, 새 측정값이 0.34 m 이상으로 확보된 뒤 풀린다. 측정값이 조금 흔들렸다고 재출발하지 않는다. 전진이 차단됐을 때 자동 방향 보정도 멈춰 제자리 보행을 방지한다. 몸체 자세는 두 초음파가 유효하고 0.30 m 이상일 때 시작하며, 진행 중 값이 끊기거나 0.26 m 미만으로 가까워지면 완만하게 복귀한다. 기존 `bow`, `crouch`도 새 목표 자세 추종·센서 기반 완료 방식에 통합했다.

정지 명령과 **정확한 위치 고정은 다르다**. 센서 차단 후 전진 속도 명령은 0에 수렴하고 READY로 돌아왔지만, 13~18초의 물리 기록에는 약 1 cm의 잔여 이동이 있다. 제자리 보행 때문에 3~4 cm 이동하던 문제는 줄였으나 5 mm 위치 고정 기준은 통과하지 않았다. `adaptive_validation.json`에 해당 결과를 별도로 기록한다. 전방 초음파와 IMU만으로 바닥 미끄러짐·절대 위치를 정확히 관측할 수 없으며, 실물의 위치 정밀도에는 관절·접촉 피드백과 비전/외부 위치 정보가 필요하다.

## 다양한 동작과 센서 상태 — RDS5160 12개 / 7.4V / 30A

사용자가 RDS5160 12개·180° 회전 범위와 7.4V·30A 전원 조건을 확인했다. `config/servo_rds5160.json`에 기록하고 URDF의 기존 30 Nm·8 rad/s 한도를 수정했다. 7.4V에서 65 kgf·cm = **6.374 Nm 정지 토크**, 0.15초/60° = **6.981 rad/s 무부하 속도**를 참고했다. 출처는 [RDS5160을 사용한 로봇 제조사의 사양표, 7쪽](https://sirenatech.com/wp-content/uploads/2019/10/ST_NinoTeen_TechnicalSpecificaitons.pdf)다. 두 최대값을 동시에 낼 수 있다는 뜻은 아니다. 사용자의 서보 실측값은 없다.

새 `control_policy_motion.json`은 앞뒤·좌우·회전·가속·달리기를 지원한다. `generated/motion/motion_demo.mp4`는 **실제 ROS 2 Humble + MuJoCo CPU 기록을 자른 41초 영상**이며 재생 속도를 바꾸지 않았다. `motion_preview.gif`는 같은 시간 속도의 16초 미리보기다. Isaac ROS 5.0 / Isaac Sim GPU 실행 기록은 아니다.

| 동작 | 실제 ROS 시험 결과 |
|---|---|
| 앞 / 뒤 이동 | 평균 +0.0991 / −0.0949 m/s, 넘어짐·FAULT 없음 |
| 좌 / 우 이동 | 평균 +0.0555 / −0.0556 m/s, 뒤쪽으로 약간의 이동 오차 있음 |
| 좌 / 우 90° 턴 | +90.20° / −90.46°, 센서 피드백으로 종료 후 READY. 중심 위치 이동 오차 약 4.4~4.7 cm |
| 달리기 모드 | 평균 0.1615 m/s, 최대 몸체 기울기 1.64°, 종료 후 READY |
| 가속·달리기·정지 버튼 | 실제 Joy 메시지 1,125개, 관절 명령 1,123개. 버튼 구간 평균 속도 0.0551 → 0.1455 → 0.1673 m/s, 정지 후 READY |
| 웅크림 / 인사 | 두 자세 수행·복귀 확인 |
| 좌 / 우 / 뒤 점프 | 준비·밀어내기·다리 접기·착지 제어 구현. **세 방향 모두 점프 성공 기준 불충족** |
| 뒷발 두 발 서기 | 경로 사전 검사에서 56° 목표가 RR 관절 범위를 벗어나 실행 거부. 실물 관절 범위·기구 간섭 확인 필요 |
| 앞발 물구나무 실험 | 무게 이동·다리 들기·75° 자세 목표·복귀 구현. 실제 기울기는 최대 약 26°, 수직 두 발 지지 시간 0초. **성공하지 않음** |

점프 성공은 전체 바닥 힘 <0.1 N, 네 발 모두 10 mm 초과, 이 상태 64 ms 이상, 지정 방향 이동 10 mm 초과, 착지 후 READY로 판정한다. 이번 시험의 가장 낮은 발 높이 최댓값은 약 0.6~1.8 mm에 그쳤다. `experimental_attempts.mp4`는 실패한 시도이며 성공 영상이 아니다. 실험 동작은 기본 비활성이고, 정지·유효 IMU·발 접촉·실제 관절 피드백을 요구한다. 정상 동작의 35° 기울기 제한을 유지한다. 앞발 서기 실험의 더 큰 기울기 범위는 명시적으로 활성화한 시뮬레이션 실험에만 적용한다.

자이로 3축(rad/s), 가속도 3축(m/s²), 자세, 발별 바닥 힘, 관절 각도를 ROS로 전달한다. `state_estimator.py`가 지지·부분 지지·공중 상태와 센서 유효성을 계산하며, 자세·자이로 값으로 보정하고 상대 yaw로 90° 턴을 제어한다. 센서가 제공하는 융합 자세가 없으면 자이로/가속도 보완 필터를 사용한다. 자이로만으로 위치·이동 속도·절대 방향·착지를 정확히 알 수 있다고 주장하지 않는다. ROS IMU 단위와 미제공 필드의 규약은 [공식 메시지 정의](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)를 따른다.

원래 MPU6050 코드는 각속도·가속도를 미제공으로 표시했다. 이제 실제 full-scale 레지스터를 읽어 SI 단위로 변환하고, 최신 DMP 패킷을 골라 전달한다. `~accel_offsets`, `~gyro_offsets`, `~sensor_mount_rpy_deg`로 개별 센서의 영점·장착 방향을 설정한다. 다른 장치의 보정값을 그대로 사용하지 않는다. 장착 방향 기본값은 센서 축과 몸체 축이 일치한다는 가정이며, 실물에서 확인하지 않았다.

현재 PCA9685 / ServoKit 경로는 **각도 명령을 보내며 실제 관절 각도나 토크를 읽지 않는다**. 시뮬레이션의 관절 피드백·발 접촉 값이 실물에도 있다고 가정하면 안 된다. 점프·수직 자세를 실물로 옮기려면 이 피드백과 전원 상태를 확보해야 한다. 사용자가 실제 서보 회전 범위를 180°로 확인했으므로 기존 PWM 코드의 180° 설정과 일치한다. 각 관절의 영점·방향·기구 간섭은 별도로 확인해야 한다.

물리 시험은 무게 약 4.09 kg, 마찰 0.8, 자체 충돌 비활성 조건이다. 토크는 속도에 따라 줄어드는 선형 근사와 30A 전원 제한을 적용했다. 서보 한 개 정지 전류 5.4A, 토크와 전류의 비례 관계는 **감도 시험용 가정**이며 제조사 실측 곡선이나 배터리 전압 강하 모델이 아니다. 별도 물리 시험에서 마찰 0.55·무게 +15%·IMU 각속도/가속도 잡음, 또는 25A·정지 전류 6A 가정으로 이동·달리기를 확인했다. 자세 quaternion은 시뮬레이션의 이상적인 값이다. 새 동작에 신경망 학습을 수행한 것은 아니다.

기본 버튼은 장치별로 다시 매핑할 수 있는 ROS Joy 인덱스다. 실물 게임패드의 인덱스는 확인하지 않았다.

| 인덱스 | 동작 |
|---|---|
| 0 / 1 | 일어서기 요청 / 정지 |
| 5 | 누르고 있을 때 이동·동작 요청 허용 |
| 4 | 누르는 동안 가속 |
| 3 | 달리기 / 일반 보행 전환 |
| 2 + 스틱 방향 | 좌·우·뒤·앞 점프 요청, 실험 활성화와 피드백 필요 |
| 6 / 7 | 인사 / 웅크렸다 복귀 |
| 8 / 9 | 뒷발 서기 / 앞발 서기 실험 요청, 기본 비활성 |
| 10 / 11 | 좌 / 우 상대 90° 턴 |
| 12 / 13 | 빠른 춤 / 애교 시작, 버튼 5를 유지. 중지·버튼 5 해제 시 복귀 |
| 4 + 12 / 4 + 13 / 4 + 6 | 선택 이전 / 다음 / 실행. 버튼 5 유지 |

```bash
# 로컬 CPU에서 하나의 동작을 재현하고 기록
bash simulation/tools/run_motion_ros_demo.sh --scenario left --case my_left
# 별도 시뮬레이터가 센서·관절 토픽을 제공할 때 제어기와 Joy 연결
ros2 launch bigspot_sim motion.launch.py
# 실험은 별도 opt-in; 실물 실행 검증을 뜻하지 않는다
ros2 launch bigspot_sim motion.launch.py enable_experiments:=true
```

로컬 기록 도구는 제어기를 자체 실행한다. 같은 ROS 도메인에 `motion.launch.py` 제어기를 중복 실행하면 안 된다. 인터랙티브 로컬 입력은 기존 `posture_simulation.py --ros --interactive --seconds 120 --policy simulation/ros2_ws/src/bigspot_sim/config/control_policy_motion.json`과 별도의 `joystick.launch.py forward_speed:=0.06`을 사용한다. Windows 게임패드의 WSL USB 전달은 검증하지 않았다. 모든 수치와 CSV는 `generated/motion/motion_validation.json`, `ros_*.csv`, `motion_validation.png`에 보존했다.

## 10배 속도 — 빠른 대각선 보행 프로파일

고속 영상은 `generated/posture/ros_fast_walk.mp4`와 같은 이름의 GIF다. 전체 웅크림·일어서기·전진·입력 끊김·정지는 `ros_fast_validation.mp4`(24초), 비교 그래프는 `fast_speed_validation.png`다. 영상은 시뮬레이션 시간과 같은 재생 속도다.

기존 한 발씩 옮기는 slow wave 대신 **오른쪽 앞+왼쪽 뒤 / 왼쪽 앞+오른쪽 뒤** 대각선 두 발을 번갈아 옮기는 trot를 추가했다. 주기는 5.594초에서 0.6초로 줄이고, 발 들림 목표 20 mm, 서기 높이 280 mm, 속도 보상 배율 1.15를 적용했다. 가속도는 0.2 m/s², 관절 명령 속도는 최대 4 rad/s로 제한한다. 기동·IMU 확인·입력 타임아웃을 유지했다. 모델 토크 상한도 기존과 같은 **가정한 6 Nm**다.

| 실제 ROS, 동일 9~21초 측정 구간 | 저속 wave | 빠른 trot |
|---|---:|---:|
| 평균 몸체 전진 속도 | 0.00984 m/s | 0.10258 m/s |
| 속도 배율 | 1 | **10.426** |
| 최종 변위, 정지 포함 24초 | 0.1282 m | 1.3347 m |
| 일어서기 이후 최대 roll/pitch | 0.52° | 0.89° |
| 넘어짐 / 최종 상태 | 없음 / READY | 없음 / READY |

속도는 실제 CSV의 연속 몸체 x 위치 차이를 시간 차이로 나누어 평균냈다. 입력 목표는 0.1 m/s다. 빠른 ROS 시험에서 관절 명령 1,497개를 수신했고 최대 관절 실제 속도는 3.06 rad/s였다. 21초에 입력 공급을 끊었고 약 22.29초에 READY로 돌아왔다. 토크가 상한에 닿는 비율은 관절×이동 샘플 기준 약 2.1%였다. 순간적으로 토크 상한까지 사용하므로 실물 모터의 연속 성능을 확인해야 한다.

60초 직접 물리 시험도 평균 0.10264 m/s, 최대 기울기 0.87°, 넘어짐 없음으로 끝났다. 마찰/질량 배율 `.55/1.0`, `.8/1.15`, `.9/.9`의 별도 24초 시험은 모두 평균 0.103 m/s 부근, 최대 기울기 0.95° 이하, 넘어짐 없음이었다. 3 Nm 상한의 추가 직접 물리 시험도 통과했으나 이것을 실물 모터 성능의 증명으로 해석하지 않는다. 모터 전류·전압 손실·발 접촉·자기 충돌·회전·후진·실물은 검증되지 않았다.

빠른 보행에서는 대각선 두 발 지지가 정상적인 계획 상태다. 이전 crawl의 '접촉 발이 3개 미만' 기준으로 평가하지 않는다. 새 ROS 기록에서 접촉 발이 2개 미만인 이동 샘플은 없었다. 이는 현재 충돌 모델의 시험 결과이며 모든 상황의 안정성 보증은 아니다.

### 이 PC에서 빠른 보행 실행

```powershell
wsl -d Ubuntu -- bash '/mnt/c/Users/redma/OneDrive/문서/ChatGPT/데일리 보고/bigspot/simulation/tools/run_fast_ros_demo.sh'
```

고속 정책은 `ros2_ws/src/bigspot_sim/config/control_policy_fast.json`이다. 기존 저속 정책과 `run_local_ros_demo.sh`도 유지한다. 물리 엔진은 MuJoCo CPU, 실제 ROS는 Humble이며 Isaac GPU 실행 결과가 아니다.

외부 조작 모드는 위 명령에 `--interactive --seconds 120 --case fast_joystick_trial`을 붙인다. 조이스틱 장치가 WSL `/dev/input`에 연결되어 있는 경우, 다른 준비된 ROS 셸에서 `ros2 launch bigspot_sim joystick.launch.py forward_speed:=0.1`로 속도를 선택할 수 있다. 자동 demo와 조이스틱을 함께 실행하지 않는다. 실제 PS4 입력·실물 모터 시험은 아직 없다. 빠른 입력 0.1 m/s만 검증했으며 0.15 m/s라는 내부 제한을 검증된 주행 속도로 간주하지 않는다.

별도 물리 브리지를 쓰는 ROS 제어 노드는 `ros2 launch bigspot_sim sim_control.launch.py policy:=<설치된_control_policy_fast.json_절대경로>`로 선택한다. CPU demo에는 제어 노드가 이미 있으므로 추가 실행하지 않는다. `check_ros_profiles.py`로 실제 rclpy에서 저속/고속 정책 선택을 검증했다.

## 저속 보행 — 영상 지적 후 수정한 wave 프로파일

최신 영상은 `generated/posture/ros_wave_walk.mp4`(보행 구간 12초), `ros_wave_walk.gif`, `ros_wave_validation.mp4`(웅크림부터 정지까지 24초)다. `wave_gait_validation.png`는 실제 발 높이·접촉·몸체 속도의 전후 비교다.

이전 영상은 뒷발이 약 2~3 mm만 들리고 몸체가 앞뒤로 움직였다. URDF 초기 각도로 만든 기본 발 배치에서 뒷발이 뒷다리 어깨 축보다 약 70 mm 앞에 있었다. 전체 질량 중심도 몸체 기준점보다 약 20 mm 뒤였다. 초기 거리 중심의 학습으로 이 문제를 놓쳤다.

새 자세는 각 다리의 앞뒤 위치를 어깨 축 기준으로 정렬하고 지지 중심을 뒤로 20 mm 옮긴다. 질량 중심 보정은 URDF 질량·관성에 따른 시뮬레이션 추정이며 실측값은 아니다. 새 slow wave crawl은 네 발이 내려간 동안 무게를 옮긴 뒤 **왼쪽 뒤 → 왼쪽 앞 → 오른쪽 뒤 → 오른쪽 앞** 순서로 한 발씩 들어 옮긴다. 계획 좌표에서 지지 발은 고정한다. 정지할 때 진행 중인 발을 착지시킨 뒤 그 발자국을 유지하며 초기 자세로 당기지 않는다. 현재 검증한 것은 느린 직선 crawl이다. 회전은 검증되지 않았으며 이 프로파일의 yaw 명령은 경고 후 무시한다.

학습은 seed=43, 동일 기본 환경에서 3세대×10회, 6개 제어값을 탐색했다. 보상에 정지 후 거리 오차, 몸체 속도 오차·후진 구간, 지지 발 수, 네 발의 들림과 기울기를 넣었다. `wave_training_history.json`에 시행별 결과를 기록했다. 신경망 학습은 아니다. `control_policy.json`의 기본값은 이 새 정책이다.

| 동일 24초 시험 | 이전 직접 물리 | 새 실제 ROS |
|---|---:|---:|
| 뒷발 높이 변화¹ | 약 2.0 mm | 16.7~19.9 mm |
| 몸체가 뒤로 움직인 이동 샘플² | 32.0% | 13.4% |
| 일어서기 이후 최대 roll/pitch | 2.01° | 0.52° |
| 최종 변위, 목표 0.13 m | 0.1101 m | 0.1282 m |
| 이동 중 접촉 발이 3개 미만인 비율 | 15.3% | 9.9% |
| 넘어짐 / 최종 상태 | 없음 / READY | 없음 / READY |

¹ 9~21초 발 중심 높이의 95백분위−5백분위다. 순간 최대 발 높이는 새 영상에서 약 24~26 mm다. ² 몸체 x 속도 <−0.002 m/s인 샘플 비율이다. 지지 발 수는 물리 엔진 충돌 접촉 기록이며 하중 센서 실측이 아니다. **일시적인 두 발 지지·몸체 후진 구간은 남아 있으므로 완전한 정적 안정 보행이나 실물 안전성을 보장하지 않는다.**

마찰/질량 배율 `.55/1.0`, `.8/1.15`, `.9/.9`의 새 별도 시험에서도 네 발이 들리고 전진·정지를 수행했다. 넘어짐 없음, 최대 기울기 0.84° 이하였다. 자기 충돌, 복잡한 지형, 큰 외란, 실물은 미검증이다. 토크 상한은 여전히 가정한 6 Nm, drive 300/8이다.

재학습은 `python simulation/tools/train_wave_crawl.py`, 영상·그래프 재작성은 `python simulation/tools/make_wave_report.py`다. 새 정책을 ROS 기본값으로 적용할 때 `wave_policy.json`을 패키지의 `config/control_policy.json`으로 복사하고 colcon 빌드한다.

## 이전 웅크림·일어서기·학습 기록 — 현재 기본 보행과 다름

이전 재생 영상: `generated/posture/ros_learned_validation.mp4`와 같은 이름의 GIF. 실제 ROS 2 Humble 노드가 `/clock`, `/bigspot/imu`, `/bigspot/joint_states`, `/cmd_vel`, `/bigspot/joint_commands`, `/bigspot/state`를 교환한다. 물리 엔진은 MuJoCo 3.15.0 CPU이며 NVIDIA Isaac 실행 결과가 아니다. Isaac ROS 5.0 목표 구성은 별도로 유지한다.

영상 순서: 0~1초 웅크림 → 1~5초 일어서기 → 5~6초 자세 안정 → 8초부터 이동 → 17초 입력 공급 중단 → 약 18.2초 정지 자세 유지. 6초에 작은 피치 방향 토크 외란을 가했다. 센서 quaternion과 각속도는 물리 상태에서 생성한다. 자세를 매 프레임 강제로 바꾸지 않으며 모터 위치 drive와 중력·바닥 접촉으로 움직인다.

| 20초 시험, 동일 기본 환경 | 학습 전 | 학습 후 직접 물리 | 학습 후 실제 ROS |
|---|---:|---:|---:|
| 이동 구간 거리, 목표 +0.09 m | -0.0787 m | +0.0997 m | +0.1033 m |
| 목표 대비 거리 오차 | 187.5% | 10.8% | 14.7% |
| 정지 완료 후 최종 변위 | -0.1470 m | +0.0684 m | +0.0722 m |
| 최종 변위의 목표 대비 오차 | 263.3% | 24.0% | 19.8% |
| 일어서기 이후 최대 roll/pitch | 1.42° | 2.01° | 2.01° |
| 넘어짐 | 없음 | 없음 | 없음 |

전진 성능은 개선됐지만 기울기는 학습 전보다 조금 커졌다. 속도 추종 오차는 남아 있다. 학습 보상은 거리 오차·기울기·실패 벌점의 합이다. seed=42, 3세대×10회이며 세대별 마찰/질량 조건은 `.8/1.0`, `.65/1.1`, `.9/.95`다. 전체 보상 곡선을 동일 조건의 성능 변화로 해석하지 않는다. 학습한 것은 몸체 이동 비율, 자세 비례/미분 gain, 속도 배율이고 신경망은 없다. `training_history.json`에 모든 시행을 기록했다.

학습에 사용하지 않은 `.55/1.0`, `.8/1.15`, `.9/.9` 마찰/질량 조합을 18초씩 검증했다. 세 조건 모두 일어서기·전진·정지를 수행했고 넘어지지 않았으며 최대 기울기는 약 2.05° 이하였다. 보행 지형·밀기 방향 전반·큰 외란을 검증한 것은 아니다.

이 시험은 원본 질량 약 4.086 kg와 관성을 사용한다. 모터 토크 상한은 **가정한 6 Nm**, drive 300/8, 보행 주기는 4초다. 실물 RDS5160의 연속 토크·전류·속도 곡선, 배터리·배선 손실, 발 접촉은 식별되지 않았다. 자기 충돌은 껐다. 이전 60초 시험은 30 Nm 가정과 별도 보행으로 수행했으므로 결과를 직접 비교하지 않는다.

이동 구간 거리는 8~17초의 첫/마지막 물리 샘플 차이다. 정지 과정에서 몸체가 뒤로 이동하므로 최종 변위를 따로 기록한다. 그래프는 처음부터 마지막까지의 변위다. **40초 실제 ROS 시험**은 이동 구간 목표 0.29 m에 0.3038 m, 정지 후 최종 변위 0.2732 m였다. 최종 거리 오차 5.8%, 최대 기울기 2.01°, 넘어짐 없음, 최종 상태 READY다. `ros_endurance.csv/json`에 저장했다. 정지 거리 오차는 남아 있다.

### 이 PC에서 재실행

설치·빌드는 이미 완료했다. Windows PowerShell에서 저장소 루트를 기준으로 실행한다.

```powershell
wsl -d Ubuntu -- bash '/mnt/c/Users/redma/OneDrive/문서/ChatGPT/데일리 보고/bigspot/simulation/tools/run_local_ros_demo.sh'
```

현재 기본값은 새 wave 보행 24초이며 종료 뒤 영상·발 접촉·CSV·JSON을 저장한다. ROS 설치는 `/opt/ros/humble`, 전용 Python은 WSL 사용자의 `~/.cache/bigspot-sim/venv`다. 다른 PC에서는 관리자 환경에서 `setup_local_ros.sh`, 사용자 환경에서 `prepare_local_ros.sh` 순서로 준비한다. 기존 Ubuntu 배포판을 업그레이드하지 않았다.

자동 탐색 재실행은 Windows Python 3.12에서 `python simulation/tools/train_posture_policy.py`다. 새 결과를 검증한 뒤 `generated/posture/learned_policy.json`을 `ros2_ws/src/bigspot_sim/config/control_policy.json`에 복사하고 colcon 빌드한다. 이 명령은 초기 정책 탐색이다. 현재 ROS 기본 설정에는 뒤에 만든 wave 정책을 저장했다.

### 조이스틱 연결

ROS 입력 어댑터를 추가했다. 일반 Joy 인덱스 0=일어서기 요청, 5=누르는 동안 이동·동작 허용, 1=정지다. 실제 PS4 버튼 이름은 확인하지 않았다. 스틱 축/버튼 번호는 파라미터로 바꿀 수 있다. raw Joy 20 Hz heartbeat와 0.3초 타임아웃, 제어 노드 0.5초 타임아웃을 적용한다.

Ubuntu에서 장치가 `/dev/input`에 연결된 환경에서는 `ros2 launch bigspot_sim joystick.launch.py`로 실행한다. Windows USB/Bluetooth 장치가 WSL에 자동으로 전달된다고 가정하지 않는다. **실제 PS4 입력과 실물 모터는 아직 시험하지 않았다.** 기록 영상은 자동 시나리오로 속도와 시작 명령을 공급한다.

자동 demo와 조이스틱을 함께 실행하면 입력이 충돌한다. 외부 조작 모드는 다음 명령으로 실행한다. 기동·속도 명령과 자동 외란을 공급하지 않는다. 14초 무입력 시험에서 CROUCH를 유지했으며 실제 ROS 관절 명령 873개를 받았다.

```powershell
wsl -d Ubuntu -- bash '/mnt/c/Users/redma/OneDrive/문서/ChatGPT/데일리 보고/bigspot/simulation/tools/run_local_ros_demo.sh' --interactive --seconds 120 --case joystick_trial
```

다른 WSL 터미널에서 아래 환경을 설정하고 조이스틱 노드 또는 수동 명령을 실행한다. 이 물리 프로세스 안에 제어 노드가 이미 있으므로 `sim_control.launch.py`를 추가 실행하지 않는다.

```bash
export ROS_DOMAIN_ID=42 PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source '/mnt/c/Users/redma/OneDrive/문서/ChatGPT/데일리 보고/bigspot/simulation/ros2_ws/install/setup.bash'
ros2 launch bigspot_sim joystick.launch.py
# 장치 없이 시험할 때는 위 조이스틱 노드 대신 아래 명령 사용
# ros2 service call /bigspot/enable std_srvs/srv/SetBool '{data: true}'
# ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.01}}'
# 속도 publisher를 종료하면 타임아웃 후 감속한다.
```

CPU 모드는 종료 후 녹화 영상을 저장한다. 현재 실시간 3D 조작 창은 제공하지 않는다. 실물은 서보 영점·배선·힘·IMU 장착 방향을 먼저 검증한다.

## 로컬 실행과 결과

저장소 루트에서 Python 3.12를 사용한다. 생성한 STEP은 `simulation/generated/cad/BigSpot_reconstructed.step`이다. 약 292 MiB이고 삼각형 평면이 남아 있어 Fusion 불러오기와 수정이 무거울 수 있다. Fusion의 파일 → 열기 → 내 컴퓨터에서 열기로 선택한다. STEP에는 조립 위치와 컴포넌트 이름이 있지만 회전 관절·설계 이력은 없다.

```powershell
python -m pip install --target simulation/generated/python_deps -r simulation/requirements-local.txt
python simulation/tools/build_model.py
python simulation/tools/audit_stl.py
# CAD를 다시 생성할 때만 실행: 수 분 이상 소요
python simulation/tools/reconstruct_step.py
# 실제 중력/접촉 보행; 영상 및 CSV/JSON 저장
python simulation/tools/local_physics.py --gait crawl --speed 0.015 --kp 300 --kv 8 --support-shift 0.6 --seconds 20 --render --case crawl_validated
# 60초 검증
python simulation/tools/local_physics.py --gait crawl --speed 0.015 --kp 300 --kv 8 --support-shift 0.6 --seconds 60 --case crawl_support06_60s
```

결과는 `simulation/generated/local/`에 저장한다. 영상은 `crawl_validated.gif`, 비교 그래프는 `validation.png`, 시간별 상태는 CSV, 측정 요약은 JSON이다. 시간은 시뮬레이션 시간이다.

| 시험 | 전진 거리 | 최대 몸체 기울기¹ | 넘어짐² |
|---|---:|---:|---|
| 기존 crawl, 20초, 몸체 이동 0 | -0.106 m | 4.163° | 없음 |
| 개선 crawl, 20초, 몸체 이동 0.6 | +0.346 m | 3.082° | 없음 |
| 개선 crawl, 60초, 몸체 이동 0.6 | +1.177 m | 3.082° | 없음 |

동일 속도 명령 0.015 m/s, kp=300, kv=8로 몸체 이동만 바꿨다. 발을 들기 전에 지지하는 다리 쪽으로 몸체를 옮긴다. IMU나 접촉 피드백은 사용하지 않는 기하 기반 보행이다. 첫 2초 서기, 마지막 2초 정지를 포함한다. 60초 시험의 이동 목표는 0.84 m였으나 결과가 약 1.18 m이므로 **속도 추종 오차가 남아 있다**.

¹ 첫 2초 이후 roll/pitch 절댓값 중 최댓값. ² 몸체 높이 0.15 m 미만 또는 roll/pitch 45° 초과로 판정. 센서 피드백에 의한 안정성 보증이 아니다.

충돌 형상은 링크별 STL의 convex hull이며 로봇 자기 충돌은 껐다. 질량·관성·토크는 원본 URDF 값, gain과 마찰은 가정이다. 원본 RDS5160 서보의 속도·전류·토크 곡선은 재현하지 않았다. 실물·Isaac Sim에 같은 성능을 보장하지 않는다. 현재 ROS 기본값은 새 학습 정책이며 Isaac 초기 자세·gain 300/8·6 Nm 가정도 새 시험 설정에 맞췄다. 물리 엔진이 다르므로 GPU 환경에서 다시 검증해야 한다.

`python simulation/tools/make_local_bundle.py`는 소스·STL·CAD·선택한 검증 로그·영상을 `../bigspot-local-results.zip`으로 묶고 CRC를 검사한다. 설치 라이브러리와 Autodesk 계정 화면은 제외한다.

## 검증한 문제와 남은 불확실성

- Trot 접지표가 2행이어서 3·4번째 다리에 접근하면 IndexError가 발생했다. 4행으로 수정했다.
- Trot stance에서 2차원 속도를 3차원 벡터와 곱해 배열 크기 오류가 발생했다. 3차원으로 수정했다.
- IK 코사인 식에서 두 다리 길이 제곱 모두를 빼야 하는데 하나를 더했다. 수정하고 작업 범위 밖 목표는 오류로 처리한다.
- RobotController가 참조하는 `BehaviorState.LAYDOWN`이 Enum에 없었다. 추가했다.
- 기존 보행 코드의 0.02초 시간 간격과 ROS 노드 60 Hz 주기도 다르다. 새 노드는 시뮬레이션 시계로 간격을 계산한다.
- 기존 IK 치수와 URDF가 다르다. 어깨 앞뒤 간격은 기존 코드 399 mm, URDF 317.310 mm이고 좌우 간격은 코드 135 mm, URDF 98.005 mm다. 상부 관절 간 거리는 코드 183 mm, URDF 약 142.74~142.85 mm다. **어느 쪽이 실물과 맞는지는 CAD·측정으로 결정해야 한다.**
- 새 IK는 URDF 좌표를 사용한다. 발 접점은 STL 바닥 3 mm 영역으로 추정했다. CAD 접점 측정 후 `build_model.py`를 갱신해야 한다.
- 원본 질량·관성·30 Nm/8 rad/s 제한은 측정되지 않은 값이다. 유지했으며, 실물 서보 성능으로 검증해야 한다.
- 초기 drive gain 40 Nm/rad, damping 2 Nm·s/rad, 바닥 마찰 0.8/0.6은 튜닝용 가정이다. 실물 식별값이 아니다.
- 로컬 초기 crawl에는 지지 방향 몸체 이동을 추가했다. 새 기동 시나리오는 IMU 피드백과 작은 피치 외란도 시험했다. 질량 중심 실측, 여러 지형·외란·자기 충돌·실물 검증은 남아 있다.
- 원본 RGB 하드웨어 의존성·PID/상태 전환 전체를 ROS 2로 옮기지 않았다. 새 패키지는 하드웨어에 명령을 보내지 않는다.

## 향후 Isaac GPU 환경 요구 조건

아래는 향후 NVIDIA GPU 환경으로 옮길 때의 구성이다. 현재 작업에서는 원격 서버를 사용하지 않는다.

- Ubuntu 24.04 x86_64, NVIDIA Ampere 이상 RTX GPU. Isaac Sim 기준 VRAM 16 GB 이상.
- Isaac ROS 5.0 공식 문서 기준 NVIDIA 드라이버 595 이상, CUDA 13.2 이상.
- GPU가 있는 서버라도 RTX 렌더러 지원 여부를 별도로 확인한다. CPU 전용·AMD·RTX 기능 없는 GPU로 이 구성을 실행할 수 있다고 가정하지 않는다.
- Docker Engine, NVIDIA Container Toolkit, SSD 여유 공간 필요. Isaac ROS 개발 환경 32 GB 이상, Isaac Sim·자산·빌드 공간은 별도로 확보한다.
- 원격 headless 실행 가능. SSH 터널이나 ROS 네트워크를 쓰며, 실행에 웹 스트리밍 공개 포트를 열 필요는 없다.
- 이 PC는 AMD RX 570, RAM 약 31.4 GiB, WSL Ubuntu 22.04다. 기존 WSL을 업그레이드하거나 GPU 드라이버를 바꾸지 않았다.

## 향후 NVIDIA 환경에서 실행 — 현재 미실행

1. 읽기 전용 확인: Windows에서 `simulation/tools/remote_check.ps1`.
2. 서버의 작업 디렉터리에 **이 변경본 전체**를 전송한다. 원본 저장소를 다시 clone하는 것만으로는 새 코드가 없다.
3. 서버에서 `python3 simulation/tools/preflight.py`. 실패하면 사양·드라이버를 먼저 해결한다.
4. 공식 문서대로 Docker Engine와 NVIDIA Container Toolkit를 준비한다. 드라이버 변경은 기존 서버 작업을 고려해 별도 결정한다.
5. 저장소 루트에서 `bash simulation/tools/setup_isaac_ros5.sh`.
6. 저장소 루트에서 다음을 실행한다.

```bash
export ISAAC_ROS_WS="$PWD"
isaac-ros activate
# 아래는 Isaac ROS 5.0 컨테이너 내부
bash simulation/tools/build_workspace.sh
cd simulation/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch bigspot_sim sim_control.launch.py
```

7. 서버에 Isaac Sim 6.0.1을 공식 배포본으로 설치한다. **다른 셸에서** 실행한다. Isaac ROS 컨테이너의 Lyrical/Python 라이브러리를 Sim 프로세스에 섞지 않는다.

```bash
# Isaac Sim 번들 Jazzy를 사용하는 별도 셸. 경로는 실제 설치 경로로 설정한다.
export ISAAC_SIM_PATH="$HOME/isaacsim"
export ROS_DOMAIN_ID=42
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_SIM_PATH/exts/isaacsim.ros2.core/jazzy/lib"
"$ISAAC_SIM_PATH/python.sh" simulation/isaac/run_sim.py --headless --frames 1800
```

기본은 몸체 고정 시험이다. URDF import, 12개 관절 방향·관절 제한·메시 크기, 토픽 흐름부터 확인한다. 이 단계는 보행 시험이 아니다.

**배포판 호환성 주의:** Isaac ROS 5.0은 Lyrical이다. 확인한 Isaac Sim 문서의 공식 브리지 지원은 Humble/Jazzy이며, 다른 native 배포판은 실험적이다. 이 구성은 Sim/Jazzy와 Isaac ROS/Lyrical 프로세스를 분리하고 표준 메시지만 교환한다. 서로 다른 배포판 사이 DDS 호환성을 여기서 보증할 수 없다. 다음 검사를 통과해야 연동이 검증된 것이다. 실패하면 최신 Sim의 native Lyrical 브리지 지원 여부를 재확인하고 실행 환경을 조정한다.

```bash
# Isaac ROS 5.0 환경의 다른 터미널에서, 저장소 루트 기준
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 simulation/tools/check_ros_bridge.py
# clock, joint_states, IMU, RGB, camera_info 수신을 실제 확인한다.
```

8. 브리지 검증 후 Isaac ROS 환경에서 perception을 시작한다.

```bash
source simulation/ros2_ws/install/setup.bash
ros2 launch bigspot_sim perception.launch.py
```

카메라는 가상 단안 카메라다. 이미지·내부 파라미터 입력만 구성했으며 AprilTag 표적·검출 성공 시험은 아직 없다. 원본 Pi 단안 카메라를 스테레오 센서로 간주하지 않는다. cuVSLAM/깊이/자율주행을 추가하려면 실제 센서 선택과 보정이 선행돼야 한다.

9. 관절 방향을 확인한 뒤에만 `--floating`으로 다시 시작해 접촉·넘어짐을 시험한다.

```bash
"$ISAAC_SIM_PATH/python.sh" simulation/isaac/run_sim.py --headless --floating
# Isaac ROS 셸
ros2 service call /bigspot/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.005}, angular: {z: 0.0}}'
# 중지: publisher를 종료하면 0.5초 뒤 감속한다. 또는 아래 서비스 사용.
ros2 service call /bigspot/enable std_srvs/srv/SetBool '{data: false}'
```

고정 몸체→정지 서기→느린 crawl→회전→trot 순서로 검증한다. 위치·자세·접촉·관절 추종 오차를 기록하고 gain·마찰·발 접점을 조정한다. GPU 실행·DDS 배포판 호환성·IMU 수신은 현재 미검증이다. 6.0.1 IMU 가속도 계산에 공개된 회전 좌표계 문제가 있어 가속도 발행은 껐으며 자세·각속도만 사용한다. 이 코드만으로 실물에 적용하지 않는다.

## Fusion 작업

Fusion 설치 후 사용자 로그인과 모델 열기가 필요하다. 암호·인증 코드는 사용자만 입력한다.
무료 개인용 조건·약관 동의를 사용자에게 확인받아 신청했다. Autodesk 계정에서 등록을 확인했으나 Chrome에서 설치 파일 다운로드가 차단됐고 앱 라이선스 갱신은 아직 확인하지 못했다. 이미 설치한 앱은 재설치할 필요가 없으며, Autodesk 안내상 반영까지 최대 30분이 걸릴 수 있다. Autodesk 계정의 개인정보와 신청 화면은 배포 묶음에 포함하지 않는다.

- `.f3d`/`.f3z` 원본을 열고 Utilities → Scripts and Add-Ins에서 `fusion/BigSpotAudit`를 추가·실행하면 몸체 질량·치수·관절을 JSON으로 저장한다. 스크립트는 형상을 변경하지 않는다.
- `python simulation/tools/audit_stl.py`로 STL과 원본 URDF를 검사하고 `ros2_ws/src/bigspot_sim/config/reconstruction.json`을 생성한다. 0.00001 mm 범위에서 정점 좌표를 묶어 모서리 연결을 검사하며 STL을 수정하지 않는다. 13개 중 몸체에서 비다양체 모서리 1개가 검출됐다. 자기 교차는 검사하지 않았고 여러 닫힌 부분이 한 STL에 포함되어 있다. 이 보고서만으로 솔리드 변환 성공이나 제조 적합성을 판단하지 않는다.
- `fusion/BigSpotReference`는 13개 STL을 mm 단위로 새 컴포넌트에 불러오고 URDF의 관절 위치·축·각도 제한을 적용한 12개 회전 관절을 만든 뒤 `generated/BigSpot_kinematic_reference.f3d`로 로컬 내보내기를 한다. **아직 실행하지 않았으므로 해당 f3d 파일은 없다.** 관절 축은 URDF에서 가져온 값으로 CAD 측정값이 아니다. 원본의 설계 이력·스케치를 복구한 솔리드 CAD가 아니다.
- `reconstruct_step.py`가 면 4개가 만나는 몸체 모서리에서 면의 연결을 분리해 닫힌 껍질을 만들고 내부 빈 공간을 보존했다. 형상 이동 없이 솔리드 23개로 저장·재불러오기 검증했다. 평면 합치기는 검증에 실패해 원래 삼각형 면을 유지했다. 원본 파라미터·스케치·설계 의도는 복구하지 않았다. 기준 평면·단면 스케치·치수·구멍을 다시 작성하는 단계와 기구 형상 개선은 남아 있다.
- 기구 개선은 어깨/무릎 축 정렬·각도 범위·배선 간섭·체결 접근·서보 지지·발 접촉부터 측정한다. 하중·재료·서보 데이터 없이 보강재 크기나 두께를 확정하지 않는다.
- 변경한 CAD에서 질량·관성·관절 원점·메시를 다시 내보내고 URDF/IK를 함께 갱신한다.

## 로컬 코드 검증

```bash
python simulation/tools/build_model.py
python simulation/tools/audit_stl.py
python -m unittest discover -s simulation/tests -v
python -m compileall -q simulation/tools simulation/fusion simulation/ros2_ws/src/bigspot_sim/bigspot_sim
```

순수 계산 테스트는 ROS 클럭을 최소 대역으로 대체했다. 실제 ROS 런타임 검증은 위의 별도 Humble/MuJoCo 실행 결과다. GPU·Fusion API 실행 검증과 구분한다.

## 근거

- [사용자가 지정한 Isaac ROS 5.0 한국어 발표](https://blogs.nvidia.co.kr/blog/isaac-ros-5-0-agentic-open-source-robotics/) — 이 환경의 웹 도구로 한국어 페이지는 열리지 않았다. 동일한 공식 영어 발표를 확인했다.
- [공식 영어 발표](https://blogs.nvidia.com/blog/isaac-ros-5-0-agentic-open-source-robotics/)
- [Isaac ROS 릴리스](https://nvidia-isaac-ros.github.io/releases/index.html)
- [Isaac ROS 플랫폼·CLI 설치](https://nvidia-isaac-ros.github.io/getting_started/index.html)
- [Isaac Sim GPU 요구 조건](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 배포판 지원](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Isaac Sim 6.0 URDF API 변경](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)
- [Isaac Sim 6.0.1 브리지 예제](https://github.com/isaac-sim/IsaacSim/tree/v6.0.1/source/standalone_examples/api/isaacsim.ros2.bridge)
- [Isaac Sim 6.0.1 IMU 생성 예제](https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/standalone_examples/api/isaacsim.sensors.experimental.physics/imu_sensor.py)
- [Isaac Sim 6.0.1 ROS IMU 포트 정의](https://github.com/isaac-sim/IsaacSim/blob/v6.0.1/source/extensions/isaacsim.ros2.nodes/nodes/OgnROS2PublishImu.ogn)
- [Isaac Sim IMU 가속도 문제 보고](https://github.com/isaac-sim/IsaacSim/issues/735)
- [Isaac ROS 5.0 AprilTag 실행 소스](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/blob/release-5.0/isaac_ros_apriltag/launch/isaac_ros_apriltag.launch.py)

기존 BigSpot/보행·IK 작업의 원저작자 lnotspotl, mike4192 및 저장소 작성자 dancal의 기여를 인정한다. 라이선스는 원본 LICENSE를 유지한다. Isaac Sim 브리지 API 패턴은 NVIDIA Apache-2.0 예제를 참고했다.
