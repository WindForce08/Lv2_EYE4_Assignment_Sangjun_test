# Module 5 — 비전 객체 추적 시스템 (EYE4 / 아이뻐)

RealSense D435 Color 영상에서 단일 색상 목표(파란 퍽)를 HSV·Contour로 찾고, 영상 중심 오차를 줄이도록
DYNAMIXEL 2개(Pan = 좌우, Tilt = 상하)를 P 제어하는 **Pan/Tilt 2축 추적 시스템**이다.
목표 미검출·인지 입력 중단·제어 통신 중단에서 정지하고, 신선한 목표 3프레임 연속 검출로 복귀한다.

> 팀 확정 조건: 발제문 기본(수평 1축)보다 확장된 **Pan/Tilt 2축 추적이 필수**다. `/target` 규약·안전·시험·제출 규칙은 발제문 그대로 따른다.

> **현재 상태 (2026-10-07)** — 코드·설정·시험 도구는 완성(IMPLEMENTED)되어 있고 아래 [22. 현재 미검증 항목](#22-현재-미검증-항목-hardware-verification-todo)은 실제 장비 시험 전이다.
> 이 문서의 명령 중 실제 장비에서 실행된 기록이 있는 것은 `results/logs/`에 근거가 있는 항목뿐이다.

목차: [1 목적](#1-프로젝트-목적) · [2 구조](#2-최종-시스템-구조) · [3 PC](#3-pc-환경-ssh-터미널) · [4 Pi](#4-raspberry-pi-환경-ros-2-runtime) · [5 D435](#5-realsense-d435) · [6 OpenCR](#6-opencr--dynamixel) · [7 설치](#7-설치-prerequisites-pi) · [8 빌드](#8-build-pi) · [9 P1](#9-problem-1-실행--인지) · [10 P2](#10-problem-2-dry-test--모터-출력-없음) · [11 OpenCR DRY](#11-opencr-dry-test--실제-보드-모터-출력-없음) · [12 방향](#12-실제-모터-방향-test) · [13 ROS 확인](#13-pi-ros-2-runtime-확인-ros_domain_id--rmw--topic) · [14 추적](#14-실제-tracking-실행) · [15 종료](#15-안전한-종료-방법) · [16 Kp](#16-problem-3-kp-test) · [17 P4](#17-problem-4-validation) · [18 bag 기록](#18-bag-record) · [19 bag 재생](#19-bag-replay) · [20 분석](#20-result-analysis) · [21 결과 위치](#21-results-위치) · [22 미검증](#22-현재-미검증-항목-hardware-verification-todo)

---

## 1. 프로젝트 목적

| 문제 | 내용 | 주요 산출물 |
|---|---|---|
| 1 | HSV·Contour 검출, 정규화 중심 오차 ex/ey, 면적비, 미검출 처리 | `perception_node`, 정상·없음·가림 이미지, 30/10 프레임 사람 대조 |
| 2 | `/target`(PointStamped) 인터페이스, 모터 출력 없는 7개 모의 입력 | `control_node`, `test_control_dry` |
| 3 | Pan/Tilt 2축 P 제어, 방향·속도·각도 제한·deadband, Kp A/B × 3회 | `control.yaml`, 추적 CSV·그래프 |
| 4 | IDLE/TRACKING/LOST/SEARCHING, 미검출·입력 timeout·통신 중단 정지, 3프레임 복귀, 목표가 없을 때 탐색(선택 심화), 지표 | `tracking_logger.py`, `analyze_tracking.py`, 펌웨어 watchdog |
| 5 | bag 기록, 입력 재처리(`/target_replay`), 결과 재분석, 팀원 재현 | `recordings/README.md` |

요구사항별 구현·검증 상태: [docs/requirements_traceability.md](docs/requirements_traceability.md)

## 2. 최종 시스템 구조

**모든 ROS 2 노드는 Raspberry Pi 한 대에서 실행한다.** 사용자 PC는 SSH 터미널로 명령을 입력하는 용도이며 ROS 2 노드를 실행하지 않는다.

```text
[사용자 PC] ── SSH (명령 입력·로그 확인·파일 전송만) ──▶ [Raspberry Pi — ROS 2 runtime]

  RealSense D435 ──USB 3──▶ realsense2_camera (공식 wrapper)
                              │ /camera/camera/color/image_raw (+ aligned depth, camera_info)
                              ▼
                           perception_node ── /target  geometry_msgs/PointStamped (x=ex, y=ey, z=area_ratio)
                              ▼                 QoS best effort, depth 1
                           control_node ───── /tracking_status  std_msgs/String (IDLE/TRACKING/LOST/SEARCHING)
                              │                 /control/pan_tilt_cmd  PanTiltCommand (rad/s, 20 Hz)
                              ▼
                           opencr_node ──USB Serial 115200──▶ OpenCR (tracking_controller_2axis)
                                                                 ├─ Pan  XM430-W350 ID 11 ┐ DYNAMIXEL
                                                                 └─ Tilt XM430-W350 ID 12 ┘ 1 Mbps, Protocol 2.0
```

노드 간 토픽은 Pi 내부 ROS 2(DDS) 통신이다. SSH는 ROS 메시지를 전달하지 않는다.

| 계층 | 파일 | 책임 |
|---|---|---|
| 인지 | `realsense_tracker/detector.py`, `perception_node.py` | HSV·Contour, ex/ey/area_ratio, 미검출 z=0, 원본 영상 시각 유지 |
| 제어 | `control_core.py`, `control_node.py` | 신선도·timeout(0.5 s), 상태, P 제어, direction(한 곳), 속도 상한, deadband, 3프레임 복귀 |
| bridge | `serial_core.py`, `opencr_node.py`, `dry_bridge.py` | ROS→시리얼, DRY/LIVE 모드 검사, 명령 나이 0.15 s, 명시적 prepare/arm, 보드 경계 정지를 정상 정지로 처리하고 `/opencr/limit` 발행, FAULT 래치 |
| 펌웨어 | `firmware/opencr/tracking_controller_2axis/` | 단위 변환, 0.5 rad/s 상한, 명령 대비 속도 감시, 엔코더 경계(두 축 정지·ARM 유지), 명령 timeout 300 ms, 모터 Bus_Watchdog 200 ms |

인터페이스·상태·timeout 상세: [docs/control_interface.md](docs/control_interface.md) · 디렉토리·담당: [directory_workflow_guide.md](directory_workflow_guide.md) · 팀 네비게이터: [팀업무_네비게이터.md](팀업무_네비게이터.md)

### 설정 파일 (Source of Truth)

| 구분 | 파일 | 읽는 주체 |
|---|---|---|
| Perception runtime | `ros2_ws/src/realsense_tracker/config/tracker.yaml` | perception_node, tools/ |
| Camera profile·topic | `ros2_ws/src/realsense_tracker/config/camera.yaml` | tracker.launch.py, perception_node, tools/ |
| Control runtime | `ros2_ws/src/realsense_tracker/config/control.yaml` | control_node (실제 추적) |
| Control DRY 시험 | `ros2_ws/src/realsense_tracker/config/control_dry.yaml` | control_dry.launch.py (고정 모의값) |
| OpenCR bridge LIVE | `ros2_ws/src/realsense_tracker/config/opencr_live.yaml` | opencr_node (실제 모터) |
| OpenCR bridge 시리얼 DRY | `ros2_ws/src/realsense_tracker/config/serial_dry.yaml` | opencr_node (DRY 펌웨어) |
| Hardware record | `config/hardware.yaml` | 사람 (펌웨어 상수의 사본·근거, 노드는 읽지 않음) |
| Verification conditions | `config/test.yaml` | 사람 (시험 전 확정 조건, 노드는 읽지 않음) |

같은 값이 두 곳에 있는 경우는 펌웨어 상수(컴파일 시 고정) ↔ `config/hardware.yaml`(사본)과 `control_dry.yaml`(시험 고정값)뿐이다. 자세한 규칙: [config/README.md](config/README.md)

## 3. PC 환경 (SSH 터미널)

- 역할: Pi에 SSH 접속해 명령 입력, 결과 파일 내려받기(`scp`), 문서 작성. **ROS 2 노드를 실행하지 않는다.**
- 필요: SSH 클라이언트. ROS 2 설치 불필요.
- 그래프·이미지는 Pi에서 파일로 저장한 뒤 PC로 복사해 확인한다 (SSH에 GUI 창 불필요).
- 이미 기록된 인지 증거(2026-10-06)는 인지 담당 노트북(Ubuntu 24.04 + ROS 2 Lyrical)에서 측정한 것이다 → 조건은 [report.md 문제 1](report.md#문제-1--hsvcontour-검출-파이프라인) 참고.

## 4. Raspberry Pi 환경 (ROS 2 runtime)

| 항목 | 값 | 상태 |
|---|---|---|
| OS | Ubuntu Server 64-bit (arm64). 발제문에 24.04 / 26.04 표기가 섞여 있음 | TODO: 실제 버전 `lsb_release -a` 기록 |
| ROS 2 | Lyrical (`/opt/ros/lyrical`) | 사용 기록 있음 (`results/logs/control/*` 시험이 Pi에서 실행됨) |
| RMW | 기본값(Fast DDS) | TODO: `echo $RMW_IMPLEMENTATION` 결과 기록 |
| ROS_DOMAIN_ID | 기본값(미설정 = 0)으로 동작. 같은 네트워크의 다른 장비와 분리가 필요할 때만 설정 | TODO: 실제 사용 값 기록 |
| 연결 | D435 → Pi USB 3 포트, OpenCR → Pi USB | D435의 Pi 연결 실행 기록은 아직 없음 |

## 5. RealSense D435

`config/camera.yaml` 주석에 실측 기록이 있다 (2026-10-06, 인지 담당 노트북):
D435 (Product ID 0x0B07, IMU 없음 — D435i 아님), serial 261322071425, firmware 5.15.1.55, RealSense ROS wrapper v4.57.7 / librealsense v2.57.7,
USB 3.2, Color 640×480@30 rgb8, 정렬 Depth 640×480 16UC1, frame_id `camera_color_optical_frame`, K = [607.571, 0, 327.340; 0, 607.530, 239.493; 0, 0, 1],
wrapper 발행 QoS RELIABLE·KEEP_LAST 1.

TODO(Pi): D435를 Pi USB 3에 연결한 상태에서 USB 속도·토픽·처리 FPS를 다시 기록한다 (Pi CPU에서 FPS가 달라질 수 있음).

## 6. OpenCR / DYNAMIXEL

| 항목 | 값 | 근거 |
|---|---|---|
| 모터 | XM430-W350 × 2 (모델 1020), Protocol 2.0, 1 Mbps | `results/logs/opencr/discovery_sdk_fix_20261006_103249/scan.log` |
| ID | Pan 11, Tilt 12 | 같은 scan.log, `docs/hardware.md` |
| Operating Mode | 1 (Velocity) | `results/logs/opencr/inspect_20261006_110144/inspect.log` |
| 원시 + 방향 | Pan 좌측, Tilt 아래쪽 (카메라 뒤에서 정면 기준) | pan/tilt_commission test_notes |
| USB 시리얼 | `/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00`, 115200 | `docs/hardware.md` |
| 펌웨어 | `tracking_controller_2axis` (기본 MODE=DRY, LIVE는 컴파일 플래그) | [firmware/opencr/README.md](firmware/opencr/README.md) |
| 안전 상수 | 0.5 rad/s, 명령 timeout 300 ms, Bus_Watchdog 200 ms, 경계(원점 기준) Pan ±1305 / Tilt ±622 counts 정지, ±1365 / ±682 FAULT, Profile_Acceleration 10 | `config/hardware.yaml` |

> ⚠ Tilt는 토크가 꺼지면 카메라 무게로 내려간다. 토크 해제(SUPPORTED_OFF)·전원 차단·reset 전에는 **항상 카메라를 손으로 지지**한다.
> 한 번에 한 프로그램만 OpenCR 포트를 사용한다 (opencr_node, miniterm, 시험 스크립트 동시 실행 금지).

## 7. 설치 prerequisites (Pi)

```bash
# ROS 2 Lyrical이 설치된 Pi에서
sudo apt install ros-lyrical-realsense2-camera python3-opencv python3-numpy python3-yaml \
                 python3-serial python3-matplotlib python3-colcon-common-extensions
# OpenCR 빌드·업로드: arduino-cli + OpenCR 보드 패키지, DynamixelWorkbench (firmware/opencr/README.md 참고)
```

- `python3-serial`: 펌웨어 시험 스크립트(`firmware/opencr/tests/*.py`)와 miniterm용. ROS bridge(`opencr_node`)는 pyserial 없이 POSIX termios를 쓴다.
- TODO(Pi): `ros-lyrical-realsense2-camera` arm64 패키지 설치 결과를 기록한다.

## 8. Build (Pi)

```bash
source /opt/ros/lyrical/setup.bash
cd ~/git/Lv2_EYE4_Assignment/lv2_module5/ros2_ws
colcon build --symlink-install --packages-select realsense_tracker_interfaces realsense_tracker
source install/setup.bash

ros2 pkg executables realsense_tracker
# 기대: perception_node, control_node, opencr_node, test_control_dry, test_serial_pty, test_serial_ros
ros2 interface show realsense_tracker_interfaces/msg/PanTiltCommand
```

`build/ install/ log/`는 Git에 올리지 않는다 (.gitignore). 이후 모든 터미널에서 위 두 `source`를 먼저 실행한다.
아래에서 `CFG=$(ros2 pkg prefix realsense_tracker)/share/realsense_tracker/config` 로 설치된 설정 폴더를 가리킨다.

## 9. Problem 1 실행 — 인지

사전조건: OpenCR/모터와 무관 (모터 출력 없음). D435가 Pi USB 3에 연결되어 있음.

```bash
# 터미널 1 (Pi): wrapper + perception만 (control/opencr 미실행)
ros2 launch realsense_tracker tracker.launch.py
# 터미널 2 (Pi): 확인
ros2 topic list | grep -E "camera|target"
ros2 topic info -v /target          # geometry_msgs/msg/PointStamped, BEST_EFFORT
ros2 topic echo /target             # 목표가 없으면 x=y=z=0
ros2 topic hz /target               # 처리 FPS (카메라 설정 30 fps와 구분)
```

장면 이미지(정상·없음·가림)와 사람 대조 평가 프레임은 [tools/README.md](tools/README.md)의 `image_capture.py --source ros`, `eval_frames.py`, `eval_score.py`로 만든다.
HSV 튜닝(`hsv_tuning.py`)은 GUI가 필요하므로 화면이 있는 컴퓨터에서 실행한다.

## 10. Problem 2 DRY test — 모터 출력 없음

사전조건: opencr_node LIVE를 실행하지 않는다. 다른 팀 노드와 섞이지 않도록 격리 도메인을 쓴다.

```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd ~/git/Lv2_EYE4_Assignment/lv2_module5
RUN=p2_dry_$(date +%Y%m%d_%H%M%S); OUT=results/logs/control/$RUN; mkdir -p $OUT

# (a) 순수 로직 단위 시험 (ROS 불필요, 32개)
PYTHONPATH=ros2_ws/src/realsense_tracker python3 -m unittest discover -s ros2_ws/src/realsense_tracker/test -v \
  2>&1 | tee $OUT/core_tests.log
# (b) control_node + DRY sink opencr_node 별도 프로세스 — 7개 모의 입력, 3프레임 복귀, 제어 종료 → bridge 정지
ros2 run realsense_tracker test_control_dry --output-dir $OUT/dry 2>&1 | tee $OUT/dry_tests.log
```

기대: `ALL REPOSITORY SEPARATE-PROCESS DRY CHECKS PASSED; SERIAL_OPENED=FALSE`

| # | 입력 | 기대 (control_dry 고정값 Kp 0.1, direction Pan -1/Tilt +1) |
|---|---|---|
| 1 | x=0, y=0, z>0 | TRACKING, pan 0, tilt 0 |
| 2 | x=+0.4, y=0, z>0 | pan −0.04 rad/s (카메라 오른쪽으로) |
| 3 | x=−0.4, y=0, z>0 | pan +0.04 rad/s |
| 4 | x=0, y=+0.4, z>0 | tilt +0.04 rad/s (카메라 아래로) |
| 5 | x=0, y=−0.4, z>0 | tilt −0.04 rad/s |
| 6 | z=0 | LOST, stop=true, 두 축 0 (즉시) |
| 7 | /target 발행 중단 | 0.5 s 후 LOST, stop=true |

수동으로 명령·상태를 보려면 `ros2 launch realsense_tracker control_dry.launch.py` (모터 출력 없음). 판정 기준은 자동 시험 (b)이다.

## 11. OpenCR DRY test — 실제 보드, 모터 출력 없음

사전조건: **모터 전원 OFF, 카메라 지지.** OpenCR에 DRY 펌웨어(기본 빌드)를 올린다 → [firmware/opencr/README.md 2절](firmware/opencr/README.md#2-dry-빌드업로드시험).

```bash
PORT=/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00
# (a) 펌웨어 단독 DRY 시리얼 시험 (MODE=LIVE이면 스스로 거부)
python3 -u firmware/opencr/tests/test_2axis_dry.py --port $PORT | tee $OUT/parser_2axis.log
# (b) ROS bridge → 실제 OpenCR(DRY): opencr_node 시리얼 DRY + control_node
ros2 run realsense_tracker opencr_node --ros-args --params-file $CFG/serial_dry.yaml \
  -p port:=$PORT -p csv_path:=$PWD/$OUT/serial_dry.csv
ros2 service call /opencr/prepare std_srvs/srv/Trigger    # 다른 터미널, CHECK→HOLD (DRY 모의)
ros2 topic echo /opencr/bridge_status                      # phase READY 확인
```

PTY(가상 시리얼)로 실제 펌웨어 코드를 돌리는 시험: [firmware/opencr/README.md 4절](firmware/opencr/README.md#4-host-side-시험-보드모터-없음).
기존 기록: `results/logs/control/serial_usb_seven_20261006_195242/` (실제 OpenCR DRY, 7개 입력 관찰. 장시간 침묵 중 원인 미확정 ROS_COMMAND_TIMEOUT 1건 — report 문제 4 참고).

## 12. 실제 모터 방향 test

사전조건 (모두 확인한 뒤에만 실행):
1. 11절 DRY 시험 통과, LIVE 펌웨어 업로드(`-DENABLE_MOTOR_OUTPUT=1`) 후 `STATUS`가 `MODE=LIVE`
2. 카메라를 손으로 받칠 수 있는 위치에 사람이 있음, 두 축을 중립 자세(Pan 3078±20, Tilt 0 mod 4096 ±20) 부근에 둠
3. opencr_node·miniterm 등 다른 포트 사용 프로그램 종료

**단계 A — 펌웨어 단독 방향 (ROS 없이)**: [firmware/opencr/README.md 3절](firmware/opencr/README.md#3-live-빌드와-단일-명령-방향-시험)의 `bench_2axis_once.py --case pan-left|pan-right|tilt-up|tilt-down|both`.
2026-10-06 기록: 6개 케이스 성공 (`results/logs/opencr/integration_live_20261006_141057/test_notes.md`).

**단계 B — 폐루프 부호 확인 (실제 영상 오차가 줄어드는지)**: 14절 절차로 추적을 켠 뒤 목표를 화면 오른쪽 → 왼쪽 → 위 → 아래에 두고
`ros2 topic echo /target`의 ex/ey가 0 쪽으로 줄어드는지 본다. 커지면 즉시 15절로 정지하고 `control.yaml`의 해당 `*_direction` 부호를 바꾼다.
(Kp부터 키우지 않는다: 검출 → 오차 부호 → 명령 → 응답 순서로 확인). 결과는 `docs/hardware.md`에 기록한다. — TODO

## 13. Pi ROS 2 runtime 확인 (ROS_DOMAIN_ID / RMW / Topic)

모든 노드가 같은 Pi에서 돌기 때문에 PC↔Pi 통신 설정은 필요 없다. 다만 **모든 SSH 터미널이 같은 ROS 설정**이어야 노드끼리 보인다.

```bash
# 모든 터미널에서 (예: ~/.bashrc에 추가)
source /opt/ros/lyrical/setup.bash
source ~/git/Lv2_EYE4_Assignment/lv2_module5/ros2_ws/install/setup.bash
# export ROS_DOMAIN_ID=<번호>                    # 필요할 때만. 설정한다면 모든 터미널에서 같은 값 (다르면 노드가 서로 안 보임)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 같은 네트워크의 다른 팀 노드·명령과 섞이지 않게 Pi 내부로 제한
echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION          # 기록용 (RMW 비어 있으면 기본 Fast DDS)

ros2 node list        # /camera/camera, /perception_node, /control_node, /opencr_node
ros2 topic info -v /control/pan_tilt_cmd   # Publisher control_node 1, Subscriber opencr_node 1
ros2 topic echo /control/pan_tilt_cmd      # 20 Hz, 정지 시 stop: true
```

## 14. 실제 Tracking 실행

사전조건 (12절 1~3 + 아래):
- `control.yaml` direction이 12절에서 확인된 값, Kp·속도 상한이 test.yaml/report에 기록된 값
- 추적 범위: 펌웨어 경계는 CHECK 때 멈춘 자세 기준 Pan ±1305 / Tilt ±622 counts(≈±115° / ±55°). 경계에 닿으면 보드가 두 축을 멈추고 ARM을 유지한다(`EVENT LIMIT STOPPED`, bridge는 FAULT가 아니라 정상 정지로 처리하고 `/opencr/limit` 발행). **실제 기구 범위는 미확인 — CHECK 전에 카메라를 기구 중앙에 둔다.**
- 추적 속도 상한 0.5 rad/s, Kp 1.0 (control.yaml). 처음에는 `-p pan_speed_limit_rad_s:=0.2 -p tilt_speed_limit_rad_s:=0.2`처럼 낮춰 방향을 먼저 확인한다.
- **목표가 없을 때 탐색** (`search_enabled: true`): 놓치면 LOST로 3 s 정지 → Pan·Tilt 전체 범위를 0.4 rad/s로 훑음 → 찾으면 추적, 못 찾으면 정지·IDLE(최대 120 s).
  추적 중 범위를 넘으면 탐색, 다시 찾은 목표가 또 범위를 넘으면 IDLE(out_of_range) — 안쪽으로 돌아오면 다시 추적. 상태 전이: [docs/control_interface.md 3.1절](docs/control_interface.md).
  **ARM은 목표가 보여 TRACKING일 때 한다** — SEARCHING 중에 ARM하면 바로 탐색 동작이 시작된다. 탐색을 멈추려면 `/control/enable false`.
- 실행 ID를 정한다: `RUN=track_01` (bag·시리얼 CSV·tracking CSV에 같은 이름)

```bash
# 터미널 1: OpenCR bridge LIVE (아직 모터 움직이지 않음 — STATUS만 전송)
ros2 run realsense_tracker opencr_node --ros-args --params-file $CFG/opencr_live.yaml \
  -p csv_path:=$HOME/runs/${RUN}_serial.csv
# 터미널 2: 카메라 + 인지 + 제어
ros2 launch realsense_tracker tracker.launch.py start_control:=true
# 터미널 3: 상태 확인
ros2 topic echo /opencr/bridge_status     # phase: BOOT
ros2 service call /opencr/prepare std_srvs/srv/Trigger   # CHECK → HOLD: 두 축 토크 ON (영속도)
#   phase: READY 확인 (실패 시 FAULT 원인 확인 — 중립 자세가 아니면 SUPPORT_AT_NEUTRAL)
# 목표를 화면 안에 두고 /tracking_status 가 TRACKING인지 확인한 뒤에만:
ros2 service call /opencr/arm std_srvs/srv/Trigger       # 구동 허가 — 이 순간부터 추적 동작
```

- 한 launch로 opencr_node까지 실행하려면 `start_opencr:=true` (서비스 호출 절차는 같음). 시리얼 로그를 따로 보려면 위처럼 분리 실행을 권장.
- 목표를 가리면(z=0) 즉시 STOP(ARM 유지) → 3 s 안에 다시 보이면 3프레임 후 자동 재개, 3 s 넘게 안 보이면 탐색. **자동 ARM은 없다**: FAULT 후에는 15절 재시작 절차.
- 상태·경계 확인: `ros2 topic echo /tracking_status` (IDLE/TRACKING/LOST/SEARCHING), `ros2 topic echo /opencr/limit` (경계 정지 `pan:+1` 등).
- 문제 3·4 기록은 터미널 4에서 `python3 tools/tracking_logger.py --run-id $RUN` (20절).

## 15. 안전한 종료 방법

| 상황 | 방법 | 결과 |
|---|---|---|
| 추적·탐색만 멈춤 (정상) | `ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"` | IDLE, stop=true → 보드 STOP, ARM 유지·토크 유지 (탐색 취소) |
| 구동 허가 해제 | `ros2 service call /opencr/disarm std_srvs/srv/Trigger` | 영속도 + DISARM, 토크 유지 |
| 비상 | 아무 노드나 Ctrl+C / `pkill -f opencr_node` | bridge 0.15 s 또는 펌웨어 300 ms timeout → 두 축 0 + DISARM (토크 유지) |
| 종료 후 토크 해제 | 아래 순서 | 카메라 지지 후에만 |

토크 해제 순서: ① 카메라를 손으로 지지 ② opencr_node 종료(포트 해제) ③ `python3 -m serial.tools.miniterm $PORT 115200` ④ `STATUS`로 상태 확인 ⑤ `SUPPORTED_OFF` 입력 → `TORQUE_OFF_CONFIRMED` 확인 ⑥ Ctrl+] 로 miniterm 종료.
포트를 닫는 것만으로는 토크가 꺼지지 않는다. FAULT 후에는 토크·정지를 가정하지 말고 지지한 뒤 SUPPORTED_OFF 또는 전원을 끈다.

재시작: FAULT·timeout 뒤에는 OpenCR reset(카메라 지지) → 중립 자세로 둠 → opencr_node 재실행 → prepare → arm. bridge는 자동 재연결·재ARM을 하지 않는다.

## 16. Problem 3 Kp test

사전조건: 12절 단계 B 완료. `config/test.yaml`의 `kp_compare` A/B 값을 **시험 전에** 기록하고 commit한다 (현재 null — TODO).

```bash
# 터미널 2를 아래처럼 바꿔 실행 (control_node만 Kp를 바꿔 단독 실행)
ros2 launch realsense_tracker tracker.launch.py            # 카메라 + 인지
ros2 run realsense_tracker control_node --ros-args --params-file $CFG/control.yaml \
  -p kp_pan:=<A_pan> -p kp_tilt:=<A_tilt>
python3 tools/tracking_logger.py --run-id kpA_01            # kpA_01..03, kpB_01..03
```

동작 순서(매 회 동일, 바닥 표시 사용): 왼쪽 3 s → 중앙 3 s → 오른쪽 3 s → 중앙 3 s. 상하도 같은 방식으로 같은 회차에 포함한다.
분석: `python3 tools/analyze_tracking.py results/logs/verification/kpA_0*.csv results/logs/verification/kpB_0*.csv --plot --problem P3`.
모터 위치를 측정하지 않으므로 그래프의 명령값을 실제 위치로 해석하지 않는다.

## 17. Problem 4 validation

| 시험 | 횟수 | 방법 | 기록 |
|---|---|---|---|
| 정상 추적 | 30 s 이상 | 14절 실행, `tracking_logger --run-id normal_01` | FPS·RMSE·유효 추적 비율 (20절) |
| 가림 후 재등장 | 2 s × 5회 | 손으로 2 s 가렸다가 시야 안 재등장. `--run-id occlusion_01` 하나에 5회 | `recovery_trials.csv` (영상으로 재등장 시각 판정) |
| 인지 입력 중단 | 1회 이상 | 인지를 따로 실행(`ros2 run realsense_tracker perception_node --ros-args --params-file $CFG/tracker.yaml -p camera_config:=$CFG/camera.yaml`)하고 Ctrl+C | 0.5 s 후 LOST·stop=true, `interruption_trials.csv` |
| 제어 통신 중단 (bridge 측) | 1회 이상 | control_node Ctrl+C | bridge ROS_COMMAND_TIMEOUT(0.15 s) → DISARM, 시리얼 CSV |
| 제어 통신 중단 (보드 측) | 1회 이상 | `pkill -9 -f opencr_node` (bridge가 DISARM을 못 보냄) | 펌웨어 300 ms timeout → 두 축 0 (보드 측 정지 근거) |

움직이지 않는 상태(DRY 또는 ARM 전)에서 끊김 처리를 먼저 확인한 뒤 실제 회전 시험을 낮은 속도·제한 범위에서 한다. 실패 회차도 모두 남긴다.
검출률·배경 오검출은 사람 대조 30/10 프레임(문제 1 도구)으로 따로 계산한다 — 노드의 detected 비율과 다르다.

## 18. bag record

사전조건: 14절로 추적이 돌고 있음, opencr_node `csv_path`와 `tracking_logger --run-id`에 같은 `$RUN`. 영상만 약 27.6 MB/s(30 s ≈ 0.8 GB) — 스크립트가 저장 공간을 먼저 확인한다.
기록 중 Pi CPU 부하로 인지 "처리 FPS" 로그가 떨어지는지 보고, 떨어지면 report에 적는다. SSH 끊김 대비로 `tmux` 안에서 실행한다.

```bash
tools/bag_record.sh success_01 --duration 20 --archive     # 소실·복귀 장면은 lost_01
```

스크립트가 하는 일: 필수 토픽 발행 확인 → 실행 중 노드 파라미터 저장(`ros2 param dump`) → `ros2 bag record`(토픽: [config/bag.yaml](config/bag.yaml) + camera.yaml) →
`ros2 bag info`·메시지 수 판정·파일별 sha256·기준 commit을 `recordings/<RUN>/`에 저장. bag은 `~/bags/<RUN>/`(Git 밖). 결과 정리·업로드: [recordings/README.md](recordings/README.md) 1·3절.

스크립트 없이 같은 기록 (같은 토픽):

```bash
RUN=success_01
ros2 bag record -o ~/bags/$RUN -s mcap --topics $(python3 tools/bag_tool.py topics record)
ros2 bag info ~/bags/$RUN && python3 tools/bag_tool.py check ~/bags/$RUN
```

## 19. bag replay

사전조건 — **실제 모터 출력 OFF** (스크립트가 ①~③을 확인하고 하나라도 어긋나면 거부한다):
① 카메라를 지지하고 OpenCR USB를 뽑거나 모터 전원 OFF ② opencr_node·control_node 종료 ③ 격리 도메인 `ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
(재생된 `/control/pan_tilt_cmd`가 실제 bridge에 닿지 않게 함. 닿더라도 bridge는 오래된 시각 명령을 거부한다.)

```bash
tools/bag_replay.sh reanalysis ~/bags/$RUN                   # B. 결과 재분석 → ${RUN}_reanalysis.csv, 실시간 ${RUN}.csv와 대조
tools/bag_replay.sh reprocess  ~/bags/$RUN                   # A. 입력 재처리 → ${RUN}_reprocess.csv, ${RUN}_reanalysis.csv와 대조
tools/bag_replay.sh reprocess  ~/bags/$RUN --images 10 --suffix _img   # 재처리 이미지 → results/images/replay/
```

스크립트 없이 같은 재현 (모든 터미널에서 ③의 export):

**A. 입력 재처리** — bag 영상만 검출기에 다시 넣고 새 결과를 `/target_replay`로 분리. 기록 당시 실제 파라미터(`recordings/$RUN/params/perception_node.yaml`, 없으면 `$CFG/tracker.yaml`)를 쓴다.

```bash
# 터미널 1
ros2 run realsense_tracker perception_node --ros-args --params-file recordings/$RUN/params/perception_node.yaml \
  -p camera_config:=$CFG/camera.yaml -p use_depth:=false -p use_sim_time:=true -p publish_debug_image:=true \
  -r /target:=/target_replay
# 터미널 2 (기록: 재처리 결과)
python3 tools/tracking_logger.py --run-id ${RUN}_reprocess --target-topic /target_replay --ros-args -p use_sim_time:=true
# 터미널 3 — rate 0.5: 검출기가 프레임을 놓치지 않게, delay 2: 구독 연결 대기
ros2 bag play ~/bags/$RUN --clock --delay 2 --rate 0.5 --topics $(python3 tools/bag_tool.py topics reprocess)
```

재처리 이미지: 터미널 2 대신 `python3 tools/eval_frames.py --scene visible --target-topic /target_replay --no-view --frames 10 --duration 8 --output-dir results/images/replay/${RUN}_reprocess_img`
(`--output-dir`를 주면 사람 대조 평가 세트·`eval_runs.csv`와 섞이지 않는다).

**B. 결과 재분석** — 저장된 `/target`·상태·명령으로 지표를 같은 코드로 다시 계산:

```bash
python3 tools/tracking_logger.py --run-id ${RUN}_reanalysis --ros-args -p use_sim_time:=true
ros2 bag play ~/bags/$RUN --clock --delay 2 --topics $(python3 tools/bag_tool.py topics reanalysis)
```

대조 (원본 영상 시각 `stamp_ns`로 프레임을 맞춤 — perception은 입력 영상 header를 그대로 복사):

```bash
python3 tools/bag_tool.py compare results/logs/verification/$RUN.csv            results/logs/verification/${RUN}_reanalysis.csv --state
python3 tools/bag_tool.py compare results/logs/verification/${RUN}_reanalysis.csv results/logs/verification/${RUN}_reprocess.csv
```

## 20. Result analysis

```bash
python3 tools/analyze_tracking.py results/logs/verification/<run_id>.csv --plot            # 화면 출력 + results/plots/<run_id>.png
python3 tools/analyze_tracking.py results/logs/verification/<run_id>.csv --append-metrics --problem P4
python3 tools/eval_score.py results/logs/perception/eval_visible_*.csv results/logs/perception/eval_empty_*.csv
```

같은 run의 실시간 CSV(`success_01.csv`)와 재분석 CSV(`success_01_reanalysis.csv`)를 같은 명령으로 계산해 차이를 report 문제 5에 적는다.
프레임 단위 일치는 `tools/bag_tool.py compare`(19절). 재처리 CSV는 상태·명령이 없어 유효 추적 비율 0 %, RMSE `nan`이 정상이다.

## 21. results 위치

```text
results/
├── images/detection/      정상·없음·가림 원본/마스크/검출 (문제 1, 2026-10-06)
├── images/evaluation/     사람 대조 평가 프레임 visible 30 / empty 10
├── images/replay/         bag 입력 재처리 이미지 (bag_replay.sh --images, 문제 5)
├── logs/perception/       장면 log.csv, 평가 CSV, perception_node 로그(처리 FPS)
├── logs/control/          ROS 제어 DRY·시리얼 bridge 시험 로그 (2026-10-06, Pi)
├── logs/opencr/           모터 스캔·상태 조회·축별 commission·2축 LIVE 단일 명령 시험
├── logs/verification/     tracking_logger CSV(<RUN>, <RUN>_reanalysis, <RUN>_reprocess), 시리얼 CSV, recovery·interruption trials
├── logs/replay/           bag 재현 실행 기록: run.txt·노드 로그·analyze·compare (bag_replay.sh, 문제 5)
├── plots/                 analyze_tracking.py 그래프 (현재 없음)
└── metrics.csv            요약 지표 (실측값만, 출처 열 포함)
```

bag 자체는 Git 밖(`~/bags`, 공유 드라이브). bag 증거(메타데이터·체크섬·실제 파라미터)는 `recordings/<RUN>/`.

## 22. 현재 미검증 항목 (HARDWARE VERIFICATION TODO)

코드가 있다고 시험이 통과된 것은 아니다. 아래는 실제 장비 결과가 아직 없다.

| 항목 | 상태 |
|---|---|
| Pi 단일 runtime에서 D435 + 인지 + 제어 + bridge 동시 실행, Pi 처리 FPS | NOT VERIFIED |
| Pan/Tilt ID 11/12, 1 Mbps, Protocol 2.0, 시리얼 경로 | VERIFIED 2026-10-06 (scan.log, hardware.md) — 장비 교체 시 재확인 |
| 원시 명령 방향 (Pan + 좌, Tilt + 아래) | VERIFIED (commission·LIVE 단일 명령) |
| 폐루프 영상 오차 감소 방향 (direction −1/+1) | NOT VERIFIED |
| 실제 기구 안전 회전 범위 (현재 경계 Pan ±1305 / Tilt ±622 counts는 요청 범위, 측정 근거 없음) | NOT VERIFIED |
| 2026-10-08 탐색(SEARCHING)·경계 정지 ARM 유지·0.5 rad/s 추적의 실제 LIVE 동작 | NOT VERIFIED — 단위 시험, 펌웨어 host 시험, DRY PTY, 모터 물리 모델 폐루프 시뮬레이션만 |
| Kp A/B 값, 속도 상한 최종값 | NOT DECIDED (test.yaml null) |
| 정상 30 s 추적, 가림 5회, /target 중단, 제어 통신 중단, 복구 시간, RMSE | NOT VERIFIED |
| opencr_node LIVE 모드(ROS → 실제 모터) | IMPLEMENTED, NOT VERIFIED (DRY 경로만 실제 보드 시험) |
| bag 기록·입력 재처리·결과 재분석, 다른 팀원 재현 | NOT VERIFIED |
| 장시간 침묵 중 ROS_COMMAND_TIMEOUT 원인 (serial_usb_seven) | 원인 미확정 |
