# Lv2 Module 5 디렉토리 및 담당 업무 가이드

> 프로젝트: `Lv2_EYE4_Assignment`
>
> 기준: 프로젝트 (1) 비전 객체 추적 시스템 발제문
>
> 목적: 팀원별 작업 영역과 디렉토리 역할을 명확히 분리하고, 발제문의 필수 Interface·검증·제출 기준과 실제 저장소 작업 위치를 연결한다.

---

# 1. 프로젝트 전체 구조

> 아래 트리는 dev/test1 기준 **실제 저장소 구조**다. (분류: runtime / test / commissioning / legacy / evidence)

```text
lv2_module5/
├── README.md                     실행·재현 안내 (Pi 단일 runtime)
├── report.md / team.md / presentation.md
├── directory_workflow_guide.md   (이 문서)
├── 팀업무_네비게이터.md
│
├── ros2_ws/src/
│   ├── realsense_tracker/                     ament_python 패키지
│   │   ├── package.xml / setup.py / setup.cfg / resource/
│   │   ├── realsense_tracker/
│   │   │   ├── detector.py          runtime  HSV·Contour 알고리즘 (노드·tools 공용)
│   │   │   ├── perception_node.py   runtime  wrapper 토픽 → /target
│   │   │   ├── control_core.py      runtime  상태·P 제어 로직 (ROS 없음)
│   │   │   ├── control_node.py      runtime  /target → /control/pan_tilt_cmd, /tracking_status
│   │   │   ├── serial_core.py       runtime  OpenCR 시리얼 상태 기계 (ROS 없음, DRY/LIVE)
│   │   │   ├── opencr_node.py       runtime  ROS ↔ USB Serial ↔ OpenCR
│   │   │   ├── dry_bridge.py        test     포트를 열지 않는 opencr_node 기본(DRY sink) 모드
│   │   │   ├── test_control_dry.py  test     문제 2 모의 입력 7개 (별도 프로세스)
│   │   │   ├── test_serial_pty.py   test     bridge ↔ DRY 펌웨어 PTY (+ STOP 복귀 반복)
│   │   │   └── test_serial_ros.py   test     control → bridge → DRY 펌웨어 PTY (ROS)
│   │   ├── launch/
│   │   │   ├── tracker.launch.py    runtime  Pi: wrapper + perception (+ control, + opencr 선택)
│   │   │   └── control_dry.launch.py test    control + DRY sink
│   │   ├── config/                 ROS 노드가 읽는 실행 설정 (Source of Truth)
│   │   │   ├── tracker.yaml  camera.yaml  control.yaml  control_dry.yaml
│   │   │   └── opencr_live.yaml  serial_dry.yaml
│   │   └── test/                   ROS 없는 단위 시험 (control_core, serial_core, detector)
│   └── realsense_tracker_interfaces/
│       ├── CMakeLists.txt / package.xml        rosidl_generate_interfaces
│       └── msg/PanTiltCommand.msg
│
├── firmware/opencr/
│   ├── README.md
│   ├── tracking_controller_2axis/   runtime  최종 2축 펌웨어 (기본 DRY, LIVE는 컴파일 플래그)
│   ├── tests/                       test     DRY 시리얼 시험, LIVE 단일 명령, host native 시험
│   ├── commissioning/               commissioning  dxl_discovery, dxl_inspect, pan/tilt commission, tilt_hold_test
│   ├── legacy/                      legacy   단일 축 파서와 그 시험 (2축 runtime 미사용, 증거 보존)
│   └── patches/, opencr_source_commit.txt
│
├── config/                          노드가 읽지 않는 기록 (README.md 참고)
│   ├── hardware.yaml                Hardware record — 펌웨어 상수 사본·근거
│   └── test.yaml                    Verification conditions — 시험 전 확정 조건
│
├── tools/                           인지 도구(HSV 튜닝·캡처·평가) + tracking_logger.py + analyze_tracking.py
├── docs/                            control_interface.md, hardware.md, requirements_traceability.md
├── results/
│   ├── images/{detection,evaluation}/
│   ├── logs/{perception,control,opencr,verification}/
│   ├── plots/
│   └── metrics.csv                  요약 지표 (실측값만)
└── recordings/README.md
```

중요:
- `/target`은 발제문 지정 `geometry_msgs/msg/PointStamped`를 사용한다.
- `PanTiltCommand.msg`는 팀이 선택한 모터 명령 Interface이며 발제문 필수 형식은 아니다.
- 팀 확정 요구사항에 따라 Pan/Tilt 2축 추적을 필수 구현한다. 발제문의 `/target` 규약과 안전·검증 기준은 그대로 사용한다.
- `realsense_tracker_interfaces`를 실제로 사용할 경우 `msg/`만 만들면 안 되며 `package.xml`과 `CMakeLists.txt`에서 `rosidl_generate_interfaces` 설정까지 완료해야 한다.

---

# 2. 담당 파트별 기본 원칙

| 영역 | 주요 담당 | 주요 업무 |
|---|---|---|
| `perception_node.py` | 인지 | HSV·Contour, ex/ey, area_ratio, 미검출 `/target` |
| `control_node.py` | 제어 | 상태 전이, Pan/Tilt 2축 P 제어, 제한·정지·복귀 |
| RealSense ROS 2 wrapper | 인지 + 통합 | D435 Color/Depth/CameraInfo 제공. 인지는 데이터 유효성, 통합은 실행·Launch·연결 담당 |
| `opencr_node.py` | 통합 + 제어 | ROS 2 ↔ Serial ↔ OpenCR |
| `launch/`, package | 통합 | 실행 구조와 의존성 |
| `config/` | 통합 관리 + 각 담당 | HSV·카메라·제어·장치·정지·시험 설정 |
| `firmware/opencr/` | 제어 | Dynamixel·보드 timeout 정지 |
| `results/` | 검증·문서화 중심 + 전체 | 이미지·CSV·상태로그·그래프·성능표 |
| `recordings/` | 통합 + 검증 | bag/영상 기록·재현 정보 |
| `report.md` | 전체 작성 + 검증 정리 | 문제 1~5 구현·설정·증거·검증·해석·한계 |
| `team.md` | 팀장 + 전체 | 4인 역할·Issue·PR·리뷰·권한·통합 확인 |
| `presentation.md` | 전체 + 검증 정리 | 구조→정상→소실/복구→정량→재현/기여→한계 |

---

# 3. `ros2_ws/`

ROS 2 Node와 실행 패키지가 위치한다. 통합 담당은 Workspace 전체 구조를 관리한다.

이 Workspace는 **Raspberry Pi에서 build·source·실행**하는 것을 기본으로 한다. 사용자 PC는 SSH 접속과 Git/파일 확인용이며, PC에서 별도의 ROS 2 Runtime을 구성하는 것을 전제로 하지 않는다.

Git에 올리지 않는 자동 생성물:

```text
build/
install/
log/
```

---

# 4. 실제 실행 구조 — PC는 SSH, Raspberry Pi가 전체 Runtime

> **팀 실제 배치 기준:** 사용자 PC는 Raspberry Pi에 SSH로 접속하여 명령을 실행하는 작업용 터미널이다. RealSense D435와 모든 ROS 2 Runtime Node는 Raspberry Pi에서 실행한다. 발제문의 PC/Pi 분산 실행 예시와 다르더라도 Interface·안전·시험·제출 기준은 그대로 지킨다.

```text
사용자 PC
- SSH 접속용 터미널
        │
        │ SSH
        ▼
Raspberry Pi
- RealSense D435 (USB 3)
- realsense2_camera
- perception_node
- control_node
- opencr_node
        │
        │ USB Serial
        ▼
OpenCR
        │
        ├─ Pan Dynamixel
        └─ Tilt Dynamixel
```

필수 확인:
- PC → Raspberry Pi SSH 접속
- Raspberry Pi의 ROS 2 환경 및 Workspace build
- RealSense D435 USB 인식과 실제 Color/Depth/CameraInfo Topic
- `perception_node` → `/target` → `control_node` → `/control/pan_tilt_cmd` → `opencr_node`의 Pi 내부 Topic 연결
- OpenCR Serial port 및 Serial 수신
- DYNAMIXEL 실제 ID / baud / protocol / direction / power
- Port 동시 점유 금지

ROS 2 Topic은 Raspberry Pi 내부 Node 간에 전달한다. DDS는 ROS 2 내부 통신 계층이지만, **PC와 Raspberry Pi 사이 DDS 통신을 별도 구성하거나 검증할 필요는 없다.**

SSH는 ROS 메시지 전달 경로가 아니라 Raspberry Pi를 원격 조작하는 수단이다.

발제문 내부의 기본 실행 장비 표기와 팀 실제 배치가 다르므로, README에는 **실제 사용한 Raspberry Pi OS / ROS 2 / RealSense / OpenCR 환경**을 측정·확인한 값으로 기록한다.

---

# 5. RealSense ROS 2 wrapper

별도 `camera_node.py`를 기본 구조에서 제거한다. D435용 ROS 2 wrapper(`realsense2_camera`)가 이미 Color/Depth/CameraInfo를 Topic으로 발행하므로, 별도 가공이나 재발행이 필요한 이유가 없다면 `perception_node.py`가 wrapper Topic을 직접 Subscribe한다.

### 담당 분리

통합 담당:
- RealSense wrapper 설치·실행 방법 정리
- `tracker.launch.py`에서 wrapper 실행 구성
- 실제 Topic 이름을 perception 설정에 연결
- 전체 Node 실행 순서와 ROS 2 연결 확인

인지 담당:
- D435 실제 입력 확인
- Color/Depth/CameraInfo Topic 확인
- encoding / frame_id / stamp 확인
- 실제 Color/Depth profile 확인
- `perception_node.py`에서 사용할 Topic과 데이터 유효성 확인

### 실행 후 확인

```bash
ros2 topic list | grep camera
ros2 topic list | grep color
ros2 topic list | grep depth
ros2 topic list | grep camera_info
```

실제 Topic 이름은 설치된 wrapper 버전/namespace 설정에 따라 달라질 수 있으므로 문서에 추측한 이름을 고정하지 않는다. `ros2 topic list`와 `ros2 topic type` 결과를 기준으로 config/README에 기록한다.

개발 기준을 안정화하기 위해 실제 확인된 Topic은 `camera.yaml`의 `color_topic`, `aligned_depth_topic`, `camera_info_topic`에 한 번 기록하고 이후에는 해당 설정 또는 Launch remap을 통해 사용한다.

추가 확인·기록:
```text
Color Image
aligned Depth
CameraInfo
encoding
frame_id
stamp
D435 serial
firmware
SDK / ROS wrapper version
USB speed
실제 Color/Depth profile
```

Depth는 Pan/Tilt 중심오차 추적의 직접 제어값이 아니라 유효 거리 확인·기록 대상으로 다룬다.

---

# 6. `perception_node.py`

### 담당
인지.

### 역할

```text
Image
→ HSV
→ 색상 Mask
→ Noise 제거
→ Contour
→ 대상 선택
→ 중심
→ ex / ey / area_ratio
→ /target
```

정규화:

```text
ex = (cx - W/2) / (W/2)
ey = (cy - H/2) / (H/2)
area_ratio = contour_area / (W × H)
```

부호:
```text
오른쪽 +
아래쪽 +
```

### `/target` 계약

```text
Topic:
/target

Type:
geometry_msgs/msg/PointStamped

point.x = ex
point.y = ey
point.z = area_ratio

미검출:
point.z = 0
```

### 필수 구현

- HSV 범위 config 분리
- 최소 면적 config 분리
- 해상도 config 분리
- 여러 Contour 후보 선택 규칙
- 원본 위 Contour/목표 중심/영상 중심 표시
- 정상 / 대상 없음 / 일부 가림 동일 설정 확인
- 미검출 시 이전 좌표 재사용 금지
- 원본 영상 timestamp 유지
- 영상 처리마다 `/target` 발행
- 미검출 정상 프레임도 `z=0` 발행
- QoS `best-effort`, depth 1부터 적용 후 호환 확인

Depth를 별도 출력할 경우 `depth_valid`와 `z_m`을 `/target.point.z`와 혼동하지 않도록 별도 규약을 둔다.


---

# 7. `control_node.py`

### 담당
제어.

### 기본 완료 범위

```text
Pan/Tilt 2축 P 추적
```

Pan/Tilt 두 축 모두 필수 제어 범위다.

### 상태

```text
IDLE
TRACKING
LOST
```

팀 상태 Topic:

```text
/tracking_status
std_msgs/msg/String
```

### 제어

```text
pan_command =
clamp(pan_direction × Kp_pan × ex,
      -pan_speed_limit,
      +pan_speed_limit)

tilt_command =
clamp(tilt_direction × Kp_tilt × ey,
      -tilt_speed_limit,
      +tilt_speed_limit)
```

필수:
- 작은 명령으로 실제 방향 확인
- 오른쪽 목표에서 실제 영상 오차가 줄어드는지 확인
- `direction = +1/-1`
- Kp/command 단위 기록
- 속도 상한
- 회전 범위
- 중심 deadband
- 범위 끝에서 바깥 방향 명령 금지
- Kp 비교는 `Kp_pan`, `Kp_tilt`를 한 세트로 묶은 설정 A/B 두 개를 시험 전에 확정

### 안전 정지

```text
z=0
→ LOST
→ 즉시 정지

/target 0.5초 미수신
→ LOST
→ 정지
```

0.5초 변경 시 이유 기록.

복구:

```text
신선한 목표 3프레임 연속 검출
→ TRACKING
```

---

# 8. Control → OpenCR Interface

발제문은 모터 명령 형식을 팀이 정의하도록 한다.

팀 기본안:

```text
Topic:
/control/pan_tilt_cmd

Type:
realsense_tracker_interfaces/msg/PanTiltCommand
```

```text
std_msgs/Header header
bool stop
float32 pan_velocity_rad_s
float32 tilt_velocity_rad_s
```

규칙:
- Velocity Mode 기준
- Pan은 좌우 추적 필수
- Tilt는 상하 추적 필수
- 두 축 모두 실제 오차가 줄어드는 방향을 검증
- ROS 2 단위 `rad/s`
- Firmware 단위 변환 위치 명시
- `stop=true`이면 속도 0
- Publish 주기 실제 구현값 기록
- 부호 실제 장착 기준 검증

실제 구현이 다른 Message를 사용하면 이 문서와 네비게이터, README, 검증 명령을 함께 수정한다.

---

# 9. `opencr_node.py`

### 담당
통합 + 제어.

### 역할

```text
ROS 2 motor command
→ Serial Protocol
→ OpenCR
→ Dynamixel
```

주요 작업:
- Serial port 연결 (`/dev/serial/by-id/...`, 115200)
- ROS 2 command Subscribe (명령 나이 0.15 s, 속도 상한 0.05 rad/s 검사)
- 단위 변환은 하지 않음: rad/s 그대로 `VEL p t` 전송 → 펌웨어가 원시 단위로 변환
- Serial Protocol 변환 (STATUS/CHECK/HOLD/ARM/VEL/STOP/DISARM)
- DRY/LIVE 모드 확인, 명시적 `/opencr/prepare`·`/opencr/arm`·`/opencr/disarm`, FAULT 래치 (자동 재ARM 없음)
- OpenCR 상태 수신
- 통신 오류 처리
- 실제 명령 주기 확인

`control_node`와 `opencr_node`는 같은 Raspberry Pi에서 실행한다. `/control/pan_tilt_cmd`의 publisher/subscriber 연결을 `ros2 topic info`와 `ros2 topic echo`로 확인하며, PC↔Pi DDS 연결은 요구하지 않는다.

---

# 10. `firmware/opencr/`

### 담당
제어.

주요 작업:
- OpenCR Firmware
- DYNAMIXEL ID / baud / protocol 확인
- Pan/Tilt 두 축 방향 확인
- 속도/각도 제한
- Serial command 처리
- 안전 정지
- watchdog
- build/upload
- Hardware 시험

통신 중단:

```text
command timeout
→ 마지막 명령 유지 금지
→ 모터 정지
```

---

# 11. `config/`

결과를 보기 전에 설정을 확정하고 실제 사용값을 보존한다.

> 실제 구현: ROS 노드가 읽는 값은 `ros2_ws/src/realsense_tracker/config/`에, 노드가 읽지 않는 기록·시험 조건은 `lv2_module5/config/`에 둔다.
> 같은 값을 여러 파일에 복사하지 않는다. 상세: `config/README.md`

| 파일 | 위치 | 주요 키 | 비고 |
|---|---|---|---|
| `tracker.yaml` | 패키지 config | `hsv_lower`, `hsv_upper`, `min_area_ratio`, `blur_ksize`, `morph_ksize`, `use_depth`, `enforce_depth_range`(기본 false), QoS | 인지 (가이드 초안의 vision.yaml 역할) |
| `camera.yaml` | 패키지 config | `width`, `height`, `fps`, `color_profile`, `depth_profile`, `color_topic`, `aligned_depth_topic`, `camera_info_topic` | launch가 해상도로 wrapper 프로파일 생성 |
| `control.yaml` | 패키지 config | `kp_pan`, `kp_tilt`, `pan/tilt_speed_limit_rad_s`, `pan/tilt_deadband`, `pan/tilt_direction`, `target_timeout_sec: 0.5`, `recovery_frames: 3` | Velocity 방식 고정. 각도 제한 키 없음 → 펌웨어 엔코더 경계 |
| `control_dry.yaml` | 패키지 config | 위와 같은 키의 고정 모의값 + opencr_node DRY sink | 문제 2 시험용 |
| `opencr_live.yaml` | 패키지 config | `port`(/dev/serial/by-id/...), `usb_serial_baudrate: 115200`, `expected_board_mode: LIVE`, `enable_live_hardware: true`, `command_max_age_sec`, `csv_path` | 실제 모터 |
| `serial_dry.yaml` | 패키지 config | 위와 같고 `expected_board_mode: DRY` | DRY 펌웨어 시험 |
| `hardware.yaml` | lv2_module5/config | 모터 모델·ID·bus baud·protocol·방향, 펌웨어 한계(0.05 rad/s, 300 ms, watchdog, 경계 counts) | 펌웨어는 YAML을 못 읽음 → .ino 상수가 SoT, 이 파일은 사본 |
| `test.yaml` | lv2_module5/config | 30 s, 가림 2 s × 5, 복구 3 s, Kp A/B(null → 시험 전 확정), 30/10 프레임 | 시험 조건 |

방향(direction)은 `control.yaml` 한 곳에서만 적용한다. 회전 범위는 실제 엔코더를 아는 OpenCR 펌웨어가 담당한다 (control_node는 위치를 적분하지 않는다).

---

# 12. `launch/`

통합 담당이 필요한 Node와 Parameter 경로, 실행 순서를 구성하고 README에 기록한다.

팀 실제 구조에서는 `tracker.launch.py`가 **Raspberry Pi에서** RealSense wrapper, `perception_node`, `control_node`를 실행하는 기준이다. `opencr_node`도 Raspberry Pi에서 실행하며, 안전상 DRY/LIVE 모드 분리가 필요하면 별도 실행 또는 명시적 파라미터로 구분한다. PC에서 ROS Runtime Node를 실행하는 Launch 구조는 사용하지 않는다.

실제 인자: `start_control`(기본 false), `start_opencr`(기본 false, `opencr_params` 기본 opencr_live.yaml), `start_realsense`(기본 true), `tracker_params`, `camera_config`, `control_params`(기본 control.yaml).
opencr_node를 실행해도 모터는 `/opencr/prepare` → `/opencr/arm` 서비스를 운영자가 호출하기 전까지 움직이지 않는다. 모의 시험은 `control_dry.launch.py`.

---

# 13. 문제 2 모의 입력 시험

모터 출력을 끄고 **발제문 기본 입력에 2축 확인 입력을 추가해 총 7개**를 검증한다.

| 입력 | 기대 결과 |
|---|---|
| `x=0, y=0, z>0` | Pan/Tilt 모두 불필요한 회전 없음 |
| `x=+0.4, y=0, z>0` | Pan 오른쪽 오차 감소 방향 명령 |
| `x=-0.4, y=0, z>0` | Pan 반대 방향 명령 |
| `x=0, y=+0.4, z>0` | Tilt 아래쪽 오차 감소 방향 명령 |
| `x=0, y=-0.4, z>0` | Tilt 반대 방향 명령 |
| `z=0` | Pan/Tilt 모두 정지 |
| `/target` 발행 중단 | 0.5초 timeout 후 Pan/Tilt 모두 정지 |

상태·명령·시간 로그를 남긴다.

---

# 14. `results/`

```text
results/
├── images/
├── logs/
├── plots/
└── metrics.csv
```

### images
- 정상
- 대상 없음
- 일부 가림
- 원본
- Mask
- 검출 결과
- 평가 프레임 근거

### logs

```text
perception/     장면 log.csv, 사람 대조 평가 CSV, perception_node 로그
control/        ROS 제어 DRY·시리얼 bridge 시험 (Pi)
opencr/         모터 스캔·상태 조회·축별 commission·2축 LIVE 단일 명령
verification/   tracking_logger CSV (문제 3·4·5), recovery_trials.csv, interruption_trials.csv
```

### plots
- Kp 비교
- 수평 ex
- RMSE
- 복구 시간
- FPS
- 검출률
- 유효 추적 비율

### metrics.csv

요약 지표 표 (실측값만, 출처 열 포함):

```csv
run_id,problem,metric,value,unit,sample_count,source,note
```

프레임별 원본 기록은 `results/logs/verification/<run_id>.csv` (`tools/tracking_logger.py`):

```csv
run_id,time_s,stamp_ns,receive_time_s,detected,ex,ey,area_ratio,state,stop,pan_command,tilt_command,command_unit
```

---

# 15. 검증·문서화 업무

시험 전:
- 대상
- 장면
- 거리
- 해상도
- Kp 설정 세트 2개(A/B): 각 세트에 `Kp_pan`, `Kp_tilt` 포함
- 반복 횟수
- 속도 제한
- 각도 제한
- deadband
- timeout
- 복구 판정
- 산식
- 로그 컬럼

필수 시험:
1. 정상 추적 30초 이상
2. 약 2초 가림 후 현재 시야 안 재등장 5회
3. `/target` 발행 중단 1회 이상
4. 제어 통신 중단 1회 이상
5. Pan/Tilt Kp 설정 세트 2종(A/B) × 각 3회 = 총 6회

복구 성공 판정:

```text
재등장 후 3초 이내 TRACKING
```

실패 회차도 포함한다.

---

# 16. 성능 지표

```text
처리 FPS
검출률
배경 오검출
수평/수직 RMSE
유효 추적 비율
복구 성공률
복구 시간
```

검출률: 목표가 보이는 최소 30프레임을 사람이 대조.
배경 오검출: 목표 없는 최소 10프레임을 사람이 대조.
수평 RMSE: `sqrt(mean(ex²))`, 검출·TRACKING 구간.
수직 RMSE: `sqrt(mean(ey²))`, 검출·TRACKING 구간.
복구 실패는 0초로 기록하지 않는다.

---

# 17. `recordings/`

대표 성공과 소실·복귀 장면 각각 10~30초.

bag 기록:
- 영상
- `/target`
- `/tracking_status`
- 모터 명령

`recordings/README.md`:
- 위치
- metadata
- 파일명·크기·hash
- Topic
- 기준 commit
- record/replay 명령
- 모터 비활성 방법
- 재현 확인자/날짜/결과

재처리는 `/target_replay` 같은 별도 Topic 사용.
실제 모터 출력은 비활성화.

---

# 18. `report.md`

문제 1~5 각각:
1. 구현 내용
2. 실행 조건
3. 결과물
4. 측정 결과
5. 해석
6. 심화 수행 여부
7. 한계

---

# 19. `presentation.md`

```text
시스템 구조
→ 정상 Pan/Tilt 2축 추적
→ 소실/정지/복귀
→ 통신 중단 정지
→ Kp/정량 결과
→ bag 재현
→ 팀 기여
→ 한계/추가 확장
```

---

# 20. `team.md`

```text
| 이름 / GitHub ID | 역할 | 담당 Issue | 병합된 본인 PR | 다른 PR 리뷰 | 구현·검증 내용 |
```

4명 모두 본인 PR 1건 이상 병합 + 타인 PR 리뷰 1건 이상.
팀장 PR도 타인 승인 후 병합.

---

# 21. 작업 영역 충돌 방지

```text
인지 → perception_node.py
제어 → control_node.py / firmware/opencr/
통합 → RealSense wrapper 실행·Launch / opencr_node.py / package / 전체 연결
검증 → results / recordings / 시험표 / 문서 증빙
```

발제문 `/target` 규약은 임의 변경하지 않는다.

---

# 22. Git 작업 흐름

현재 팀 운영:

```text
dev/<이름>
→ 구현·시험
→ push
→ Pull Request
→ Review / 수정
→ 팀장 Merge
→ main 동기화
```

Issue 완료 조건, PR 실행 증거, 의미 있는 리뷰, 팀장 최종 병합을 연결한다.

---

# 23. 전체 시스템에서 담당 위치

```text
D435 Color
   ↓
인지
HSV/Contour → ex/ey/area_ratio
   ↓
/target PointStamped
   ↓
제어
IDLE/TRACKING/LOST
Pan/Tilt P control
   ↓
motor command
   ↓
통합
Raspberry Pi 내부 ROS 2 Topic
Pi ↔ OpenCR USB Serial
   ↓
OpenCR
   ├─ Pan Dynamixel
   └─ Tilt Dynamixel
        ↓
카메라 Pan/Tilt 회전
        ↺

검증
Interface
→ 정상 30초
→ 가림 5회
→ /target 중단
→ control 통신 중단
→ Kp 2종×3회
→ 지표
→ bag 재현
→ 문서/PR 증빙
```

---

# 24. 최종 작업 원칙

```text
시험 조건 사전 확정
↓
HSV·Contour
↓
/target PointStamped
↓
7개 모의 입력(발제 5 + 2축 2)
↓
Pan/Tilt 2축 P 제어
↓
속도·각도·deadband
↓
Raspberry Pi ROS 2 ↔ OpenCR 통합
↓
미검출·0.5초 input timeout 정지
↓
보드 측 통신중단 정지
↓
3프레임 연속 검출 복귀
↓
Kp 2종×3회
↓
정상 30초 / 가림 5회 / 중단 2종
↓
30프레임 검출 + 10프레임 오검출 대조
↓
FPS / RMSE / 복구 지표
↓
bag 모터 비활성 재현
↓
report / team / presentation / 제출 증빙
↓
Depth/SEARCHING 등 추가 확장
```

Pan/Tilt 2축 결과를 필수 결과로 기록한다.
