# OpenCR 펌웨어 (담당: 제어, 빌드·업로드는 통합과 공동)

모든 빌드·업로드·시리얼 작업은 OpenCR가 USB로 연결된 **Raspberry Pi**에서 SSH로 수행한다.
한 번에 한 프로그램만 포트를 연다 (opencr_node, miniterm, 시험 스크립트 동시 사용 금지).

## 1. 파일 분류

| 경로 | 분류 | 역할 |
|---|---|---|
| `tracking_controller_2axis/` | **runtime** | 최종 Pan/Tilt 펌웨어. 기본 MODE=DRY, `ENABLE_MOTOR_OUTPUT=1`일 때만 MODE=LIVE |
| `tests/test_2axis_dry.py` | test | 실제 보드 MODE=DRY 시리얼 시험 (MODE=LIVE면 스스로 거부) |
| `tests/bench_2axis_once.py` | commissioning | LIVE 단일 VEL 1회 후 침묵 → 300 ms timeout 관찰 (`--execute-live` 필수) |
| `tests/test_integrated_native.cpp`, `tests/native_stubs/` | test | 실제 .ino를 PC/Pi host에서 컴파일한 시나리오 시험 (가짜 Arduino/Workbench) |
| `tests/native_pty.cpp` | test | DRY 펌웨어를 PTY에 연결하는 host 실행기 (ROS `test_serial_pty`, `test_serial_ros`가 사용) |
| `commissioning/dxl_discovery/` | commissioning | 모터 ID·baud·프로토콜 스캔 (레지스터 쓰기 없음) |
| `commissioning/dxl_inspect/` | commissioning | 모드·토크·위치 등 상태 조회 |
| `commissioning/pan_commission/`, `tilt_hold_test/`, `tilt_commission/` | commissioning | 축별 첫 구동·Tilt 부하 유지 시험 (2026-10-06 기록의 원본 펌웨어) |
| `legacy/tracking_controller/`, `legacy/tests/test_parser*.py` | legacy | 단일 속도 파서 기준 버전과 그 시험. **2축 runtime에 사용하지 않음** (`parser_baseline` 로그 근거로 보존) |
| `patches/` | build | ARM GCC 14의 Arduino min/max 매크로 충돌 패치 (DynamixelWorkbench, OpenCR SDK) |
| `opencr_source_commit.txt` | build | 패치를 적용한 OpenCR 보드 패키지 source commit |

Arduino는 스케치 디렉터리 하나만 빌드한다. `native_stubs`는 Arduino 라이브러리로 설치하지 않는다.
상수(ID·baud·한계·timeout)의 근거와 사본: [`../../config/hardware.yaml`](../../config/hardware.yaml) — 상수를 바꾸면 같은 commit에서 함께 고친다.
시리얼 프로토콜·안전 계층: [`../../docs/control_interface.md`](../../docs/control_interface.md)

## 2. DRY 빌드·업로드·시험

사전조건: **모터 전원 OFF, 카메라 지지.** miniterm/opencr_node 종료.

```bash
REPO=$HOME/git/Lv2_EYE4_Assignment
PORT=/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00
BUILD=$HOME/pa-opencr-build/build/tracking_controller_2axis_dry
LOG=$REPO/lv2_module5/results/logs/opencr/integration_dry_$(date +%Y%m%d_%H%M%S)
mkdir -p "$BUILD" "$LOG"; set -o pipefail
arduino-cli compile --clean --fqbn OpenCR:OpenCR:OpenCR --build-path "$BUILD" \
  "$REPO/lv2_module5/firmware/opencr/tracking_controller_2axis" 2>&1 | tee "$LOG/build.log"
# 컴파일 성공 후에만
opencr_ld "$PORT" 115200 "$BUILD/tracking_controller_2axis.ino.bin" 1 2>&1 | tee "$LOG/upload.log"
# OpenCR reset 후
python3 -u "$REPO/lv2_module5/firmware/opencr/tests/test_2axis_dry.py" --port "$PORT" 2>&1 | tee "$LOG/parser_2axis.log"
```

기대: `ALL TWO-AXIS DRY CHECKS PASSED`. 기록: `results/logs/opencr/integration_dry_20261006_140439/`.

## 3. LIVE 빌드와 단일 명령 방향 시험

> **2026-10-08 펌웨어 변경 (추적 0.5 rad/s, 탐색용 경계 정지) — 보드의 이전 LIVE 펌웨어는 다시 빌드·업로드해야 한다.**
> | 항목 | 이전 | 현재 | 이유 |
> |---|---|---|---|
> | `MAX_RAD_S` | 0.10 rad/s | 0.5 rad/s | 추적 속도 요구 0.5 rad/s (호스트 `MAX_VELOCITY_RAD_S`와 같음) |
> | 속도 감시 | 절대 5단위(≈0.12 rad/s) | 최근 목표 \|goal\| + 3단위 | 0.5 rad/s(21단위) 정상 추적이 `FAULT UNEXPECTED_SPEED`가 되지 않게 |
> | `Profile_Acceleration` | 1 (≈0.37 rad/s²) | 10 (≈3.7 rad/s²) | 0.5 rad/s 정지 ≈1.3 s → ≈0.13 s (500 ms 정지 확인·경계 여유 안) |
> | 경계 정지 | Pan ±1345 / Tilt ±662, 이벤트에 축 정보 없음 | Pan ±1305 / Tilt ±622 (OUTER − 60), `EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN\|TILT DIR=±1` | 0.5 rad/s에서 정지 거리 여유, 호스트 탐색 반환점·bridge 정상 정지 처리 |
>
> 근거: host 시험(`tests/test_integrated_native.cpp` — 축·방향 이벤트, 경계 후 `ERR STOPPING`·재개, 0.5 rad/s, 명령 초과 속도 FAULT), DRY PTY,
> 모터 물리 모델 폐루프 시뮬레이션(경계 넘은 거리 최대 27 counts < 여유 60). **실제 LIVE 동작은 미확인** — 처음에는 속도 상한을 낮춰 방향·경계부터 확인한다.

사전조건: 2절 통과. 카메라를 받칠 사람이 옆에 있음. 두 축을 중립(Pan 3078±20, Tilt 0 mod 4096 ±20) 부근에 둠.
전원·reset은 토크를 풀 수 있으므로 내내 지지한다. DYNAMIXEL 모드를 바꾸지 않는다. DRY와 다른 빌드 폴더를 쓴다.

```bash
BUILD=$HOME/pa-opencr-build/build/tracking_controller_2axis_live
LOG=$REPO/lv2_module5/results/logs/opencr/integration_live_$(date +%Y%m%d_%H%M%S)
mkdir -p "$BUILD" "$LOG"
arduino-cli compile --clean --fqbn OpenCR:OpenCR:OpenCR \
  --build-property 'compiler.cpp.extra_flags=-DENABLE_MOTOR_OUTPUT=1' --build-path "$BUILD" \
  "$REPO/lv2_module5/firmware/opencr/tracking_controller_2axis" 2>&1 | tee "$LOG/build.log"
opencr_ld "$PORT" 115200 "$BUILD/tracking_controller_2axis.ino.bin" 1 2>&1 | tee "$LOG/upload.log"
```

업로드 후 `python3 -m serial.tools.miniterm $PORT 115200`에서 `STATUS` → **`MODE=LIVE`** 확인 (DRY로 나오면 플래그가 적용되지 않은 것 — 중단하고 빌드 로그 확인).
`CHECK` → `HOLD` → `EVENT STOPPED TORQUE_RETAINED` → `STATUS`가 `DISARMED GOAL=0,0 VEL=0,0 TORQUE=1,1`. 처짐·진동이 없는지 본 뒤 Ctrl+]로 닫는다 (토크 유지됨, 자리를 비우지 않는다).

```bash
python3 -u "$REPO/lv2_module5/firmware/opencr/tests/bench_2axis_once.py" --port "$PORT" --execute-live --case zero 2>&1 | tee "$LOG/zero.log"
# zero 성공 후 한 번에 하나씩: pan-left, pan-right, tilt-up, tilt-down, both (각 성분 ±0.024 rad/s = 원시 1단위)
```

| case | 기대 동작 (카메라 뒤에서 정면 기준) |
|---|---|
| pan-left / pan-right | Pan 좌/우, Tilt 유지 |
| tilt-up / tilt-down | Tilt 위/아래, Pan 유지 |
| both | Pan 우 + Tilt 아래 |

방향이 틀리거나 처짐·진동·FAULT가 있으면 중단한다. 각 시험은 VEL 1회 후 1초 침묵으로 300 ms 명령 timeout과 DISARM을 확인한다.
2026-10-06 기록: 6개 케이스 성공 (`results/logs/opencr/integration_live_20261006_141057/test_notes.md`, `zero_retry_notes.md`).
세션 끝: 카메라 지지 → miniterm `SUPPORTED_OFF` → `TORQUE_OFF_CONFIRMED`. 다음 세션은 reset + CHECK/HOLD부터. FAULT 후 자동 재ARM 없음.

## 4. Host-side 시험 (보드·모터 없음)

```bash
cd $REPO/lv2_module5/firmware/opencr/tests
g++ -std=c++17 -Wall -Wextra -Werror -I native_stubs test_integrated_native.cpp -o /tmp/native-dry && /tmp/native-dry
g++ -std=c++17 -Wall -Wextra -Werror -DENABLE_MOTOR_OUTPUT=1 -I native_stubs test_integrated_native.cpp -o /tmp/native-live && /tmp/native-live
# ROS PTY 시험용 DRY 펌웨어 실행 파일 (Linux)
g++ -std=c++17 -Wall -Wextra -I native_stubs native_pty.cpp -o $HOME/pa-opencr-build/build/native_bridge/opencr_dry_pty
ros2 run realsense_tracker test_serial_pty --firmware-executable $HOME/pa-opencr-build/build/native_bridge/opencr_dry_pty \
  --output-dir <LOG>/pty --cycles 20 --tick-sec 0.01
ros2 run realsense_tracker test_serial_ros --firmware-executable $HOME/pa-opencr-build/build/native_bridge/opencr_dry_pty \
  --output-dir <LOG>/serial_ros
```

기대: `PASS native controller scenarios MODE=DRY` / `MODE=LIVE_STUB`, `ALL SERIAL PTY CORE CHECKS PASSED`, `ALL SERIAL ROS PTY CHECKS PASSED`.
검사 범위: 두 값 중 하나라도 잘못되면 모터 쓰기·타이머 갱신 없음, 침묵·오류·STATUS에서도 timeout, 반복 ARM 거부, STOP/DISARM, millis rollover,
두 번째 축 쓰기 실패 시 두 축 0 시도 후 FAULT 고정, FAULT 후 버스 중단, 피드백 실패, 위치 경계 정지, DRY에서 버스 호출 0회.
host 시험은 USB·ARM toolchain·실제 SDK 지연·중력 부하·정지 거리를 재현하지 않는다 — 하드웨어 승인 근거로 쓰지 않는다.

## 5. 아직 하지 않은 하드웨어 시험

위치 경계 정지, 실제 정지 지연(마지막 명령 → 0 쓰기 → 속도 0 피드백), 모터 버스 통신 단절, 보드 멈춤 시 Bus_Watchdog, 피드백 실패.
모터 케이블을 뽑거나 한계까지 반복 jog하는 즉흥 시험은 하지 않는다. 별도 절차를 설계한 뒤 수행한다.
