# 큰 춤·앉아서 박수 검증

2026-10-09. 실행: 로컬 WSL ROS 2 Humble + MuJoCo 3.15 CPU. 실물·Isaac GPU 실행은 미검증이다.

아래는 큰 춤·박수를 추가한 시점의 기록이다. 현재 기본 버튼 12와 30종 목록은 [빠른 스텝 춤 검증](FUNK.ko.md)을 따른다. 큰 춤과 박수는 계속 선택할 수 있다.

| 항목 | 실제 ROS 물리 기록 |
|---|---|
| 큰 춤 `dance_show` | 높이 변화 69.9 mm, roll 변화 17.5°, 좌우 이동 폭 약 63 mm |
| 직전 기본 빠른 춤 대비 | 높이 변화 2.38배, roll 변화 2.97배 |
| 템포 | 90 BPM, 6박자 몸체 주기 4초. 기존 빠른 춤은 2.67초로 선택 목록에 유지 |
| 박수 `seated_clap` | 몸을 뒤로 옮겨 약 35° 기울임 → 앞발 들기 → 박수 3회 → 착지 확인 → 웅크리기 → 기립 |
| 실제 앞발 접촉 | 3회, 최대 1.06 N. 앞발 사이 거리만으로 성공 판정하지 않음 |
| 박수 중 앞발 높이 | 최소 43.9 mm. 발 벌림 약 200 → 21 mm |
| 박수 중 뒷다리 지지 여유 | 측정한 접촉면에 대한 무게중심 여유 최소 27.8 mm |
| 동작 시간 | 큰 춤 경로 22초, 박수 경로 43.5초. 안정 확인 후 각각 약 22.4·43.9초에 완료 |
| 관절 명령 속도 | 모든 새 ROS 기록에서 1.2 rad/s 이하 |

큰 춤 및 박수 모두 기본 조건과 질량 +15%·마찰 0.55·자이로/가속도 잡음 조건에서 넘어짐·FAULT 없이 완료했다. 큰 춤은 접촉 힘 >1 N인 지지 발 최소 3개를 유지했다. 박수는 두 뒷다리 아래쪽과 발의 CAD 접촉면으로 지지한다. 몸통 뒤쪽의 바닥 접촉 힘은 0이다. STL 접촉 형상으로 기립 경로 높이를 계산했으며 관절 제한을 넓히거나 발 패드를 추가하지 않았다.

앞발 아래쪽 CAD 메시 두 개 사이의 충돌만 켰다. MuJoCo의 볼록 메시 접촉이며 다른 링크 사이의 자체 충돌은 꺼져 있다. 전체 기구 간섭·실제 서보 배선·각도 교정 검증은 아니다. 질량, 7.4 V / 30 A 전원 및 토크·속도·전류 모델은 미측정 가정이다. 자세 quaternion은 이상적인 값이므로 실물 자이로 정확도 검증도 아니다.

14개 실제 ROS 시험: 정상 동작 5개(두 동작 기본·별도 조건 및 박수 버튼 실행), 복귀 5개(박수 취소·음파 단절·데드맨 해제, 춤 정지·데드맨 해제), 중요 센서 단절 4개다. IMU 단절 0.216초, 관절·뒷다리 지지 정보 단절 각 0.200초, 발 접촉 단절 0.104초 안에 FAULT로 전환했다. 이후 관절 명령 변동은 0이었다. **마지막 각도 명령 유지이며 모터 전원을 끄는 비상 정지는 아니다.** 복귀 요청은 앞발을 내려놓고 두 앞발의 착지를 확인한 뒤 몸체 자세를 바꾼다. 코드 검사 77개도 통과했다.

일반 ROS Joy 인덱스 기준으로 버튼 5를 유지한다.

| 입력 | 동작 |
|---|---|
| 5 + 8 / 12 / 13 | 박수 / 큰 춤 / 애교 |
| 5 + 4 + 12 / 13 | 선택 목록 이전 / 다음 |
| 5 + 4 + 6 | 선택한 동작 한 번 실행 |
| 1 또는 5 해제 | 진행 동작 복귀 요청 |

선택 목록은 춤 6종, 애교 4종, 몸체 자세 15종, 박수 1종, 턴 2종으로 28종이다. 실제 게임패드 버튼 번호는 미확인이다. ROS 1 실물 PWM 경로에는 새 동작을 연결하지 않았다. 현재 실물은 관절·발 접촉·뒷다리 지지 피드백이 없어 박수 시작 조건을 충족하지 못한다. 현재 Isaac 브리지에도 발 접촉·지지·초음파 정보 추가 및 GPU 시험이 필요하다. 정보가 없으면 동작 시작을 거부한다.

```bash
bash simulation/tools/run_show_ros_demo.sh --scenarios dance_show holdout_dance_show seated_clap holdout_seated_clap clap_buttons
bash simulation/tools/run_show_ros_demo.sh --scenarios clap_cancel clap_sonar_dropout clap_deadman show_buttons show_deadman clap_imu_dropout clap_joint_dropout clap_seat_dropout clap_contact_dropout
python simulation/tools/recheck_show_traces.py
python simulation/tools/make_show_report.py --validate-only
# 별도 시뮬레이터가 센서·관절 피드백을 제공할 때 직접 요청
ros2 topic pub --once /bigspot/action std_msgs/msg/String '{data: seated_clap}'
```

`generated/show/show_validation.json/png`, 14개 ROS CSV·센서 JSON에 결과를 저장했다. `dance_show.mp4` 23초, `seated_clap.mp4` 45초 영상은 정상 재생 속도다. 카메라는 고정했고, 박수 영상은 저장한 실제 물리 상태를 앞쪽 시점에서 다시 렌더링했다. 원래 물리 상태와 재생 속도는 바꾸지 않았다. `show_preview.gif`는 두 동작의 23초 발췌다. 실패한 초기 자세·기립 경로 기록도 작업 폴더에 보존했다.
