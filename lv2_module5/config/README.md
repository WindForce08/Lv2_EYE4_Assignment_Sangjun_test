# config — 설정 위치와 Source of Truth

발제문 제출 구조의 `config/`는 "HSV·카메라·제어·장치·정지·시험 설정"을 보여 주는 곳이다.
ROS 노드가 실제로 읽는 설정은 패키지 안에 두고 여기로 복사하지 않는다 (발제문: "설정이 패키지 안에 있으면 중복 복사하지 않고 경로를 연결").

| 구분 | 파일 | 읽는 주체 | 주요 내용 |
|---|---|---|---|
| Perception runtime | [`../ros2_ws/src/realsense_tracker/config/tracker.yaml`](../ros2_ws/src/realsense_tracker/config/tracker.yaml) | perception_node, tools/ | HSV, 최소 면적, 블러·모폴로지, 깊이 기록·선택 gate, QoS |
| Camera | [`../ros2_ws/src/realsense_tracker/config/camera.yaml`](../ros2_ws/src/realsense_tracker/config/camera.yaml) | tracker.launch.py, perception_node, tools/ | 해상도·FPS, wrapper 토픽 이름, D435 실측 기록 |
| Control runtime | [`../ros2_ws/src/realsense_tracker/config/control.yaml`](../ros2_ws/src/realsense_tracker/config/control.yaml) | control_node | Kp, 축별 속도 상한·deadband·direction, timeout 0.5 s, 복귀 3프레임, 탐색(search_*) |
| Control DRY test | [`../ros2_ws/src/realsense_tracker/config/control_dry.yaml`](../ros2_ws/src/realsense_tracker/config/control_dry.yaml) | control_dry.launch.py | 문제 2 모의 시험 고정값 (실제 추적에 쓰지 않음) |
| OpenCR bridge LIVE | [`../ros2_ws/src/realsense_tracker/config/opencr_live.yaml`](../ros2_ws/src/realsense_tracker/config/opencr_live.yaml) | opencr_node | 포트, USB 115200, LIVE 확인 플래그, 명령 나이 0.15 s |
| OpenCR bridge DRY | [`../ros2_ws/src/realsense_tracker/config/serial_dry.yaml`](../ros2_ws/src/realsense_tracker/config/serial_dry.yaml) | opencr_node | DRY 펌웨어 전용 |
| Hardware record | [`hardware.yaml`](hardware.yaml) | 사람 | 모터 모델·ID·bus baud·protocol·방향, 펌웨어 안전 상수 (사본) |
| Verification conditions | [`test.yaml`](test.yaml) | 사람 | 30 s, 가림 2 s × 5, 복구 3 s, Kp A/B, 평가 프레임 30/10 |
| Bag record/replay | [`bag.yaml`](bag.yaml) | `tools/bag_tool.py` (bag_record.sh·bag_replay.sh가 호출) | 기록·재생 토픽, 기대 주기, 기간 10~30 s. 카메라 토픽 이름은 camera.yaml을 읽음 (중복 없음) |

## 중복 값 규칙

| 값 | Source of Truth | 사본 | 동기화 방법 |
|---|---|---|---|
| 모터 ID, bus baud, 속도 상한 0.5 rad/s, 명령 timeout 300 ms, watchdog, 경계 counts, 가속 프로파일 | 펌웨어 `tracking_controller_2axis.ino` 상수 (YAML을 읽을 수 없음) | `hardware.yaml` | 펌웨어 상수를 바꾸는 commit에서 hardware.yaml도 함께 수정 |
| 속도 상한 0.5 rad/s (호스트 검사) | `control_core.MAX_VELOCITY_RAD_S` | serial_core·dry_bridge가 import | 코드 한 곳 |
| direction (Pan −1, Tilt +1) | `control.yaml` | `control_dry.yaml` (시험 고정값) | bridge·펌웨어에는 두지 않음 |
| target timeout 0.5 s, 복귀 3프레임 | `control.yaml` | `test.yaml` (시험 조건으로 기록) | 바꾸면 둘 다 수정하고 report에 이유 기록 |
| 해상도·FPS | `camera.yaml` | 없음 (launch가 읽어서 wrapper 프로파일 생성) | — |
