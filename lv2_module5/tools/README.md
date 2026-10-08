# 인지 파트: 검출 노드와 도구 (담당: 인지 김상화)

RealSense 영상에서 파란색 목표(퍽)를 HSV 색상으로 찾아, 화면 중심 대비 어긋남(`ex`, `ey`)을 계산합니다.

| 위치 | 역할 |
|---|---|
| `ros2_ws/src/realsense_tracker/realsense_tracker/detector.py` | 검출 알고리즘 (ROS·카메라와 무관) — 노드와 도구가 함께 사용 |
| `ros2_ws/src/realsense_tracker/realsense_tracker/perception_node.py` | ROS 2 인지 노드 — `/target` 발행 |
| `ros2_ws/src/realsense_tracker/config/tracker.yaml` | 인지 파라미터 (`perception_node` 항목) |
| `ros2_ws/src/realsense_tracker/config/camera.yaml` | 카메라 해상도·fps·profile, **카메라 토픽 이름** (launch·노드·도구가 사용) |
| `tools/hsv_tuning.py` | HSV 범위 튜너 (트랙바) → `tracker.yaml`에 저장 |
| `tools/image_capture.py` | 정상·미검출·가림 장면 검출 결과 저장 → `results/` |
| `tools/common.py` | 도구 공통 (경로, 설정 읽기, RealSense 카메라) |
| `tools/eval_frames.py` | 검출률·배경 오검출 평가 프레임 저장 (ROS) → `results/` + 판정 목록 CSV. `--output-dir`: bag 재처리 이미지용(평가 세트와 분리) |
| `tools/eval_score.py` | 사람이 판정한 CSV로 검출률·배경 오검출 집계 |
| `tools/tracking_logger.py` | 문제 3·4·5: /target·상태·명령을 프레임별 CSV로 기록 (실시간·bag 재분석 공용, ROS) |
| `tools/analyze_tracking.py` | 문제 3·4·5: 처리 FPS·RMSE·유효 추적 비율·소실/복귀 구간·그래프 (ROS 없음) |
| `tools/bag_record.sh` | 문제 5: bag 기록 + 실제 파라미터·info·메시지 수 확인·sha256 → `recordings/<RUN>/` (Pi, ROS) |
| `tools/bag_replay.sh` | 문제 5: 모터 OFF 확인 후 A 입력 재처리 / B 결과 재분석 → CSV·로그·자동 대조 (ROS) |
| `tools/bag_tool.py` | 문제 5: 기록 토픽 목록, bag 확인(`check`), CSV 프레임 대조(`compare`) (ROS 없음, `config/bag.yaml`) |

검출 순서: 영상 → 블러 → HSV 변환 → 색 마스크 → 잡음 제거 → 외곽선 → **가장 큰 덩어리 1개 선택** → 중심 계산

---

## 1. `/target` 메시지 약속 (발제문 지정 — 이름·형식 임의 변경 금지)

`geometry_msgs/msg/PointStamped`

| 필드 | 값 | 미검출일 때 |
|---|---|---|
| `point.x` | `ex` = 화면 중심 기준 가로 어긋남, −1(왼쪽 끝) ~ +1(오른쪽 끝) | 0 |
| `point.y` | `ey` = 화면 중심 기준 세로 어긋남, −1(위쪽 끝) ~ +1(아래쪽 끝) | 0 |
| `point.z` | `z` = 목표 넓이 ÷ 화면 넓이 (검출되면 항상 0보다 큼) | **0 → 미검출 신호** |
| `header` | 입력 컬러 영상의 시각·좌표계 | 같음 |

- 제어는 **`z == 0`이면 미검출**로 판단합니다. (`x = y = 0`만으로는 "정중앙"과 구분되지 않음)
- 영상이 들어올 때마다 1번씩 발행합니다. 영상이 끊기면 발행도 멈춥니다.

## 2. 카메라 토픽 확인 (RealSense ROS 2 wrapper)

카메라는 별도 노드 없이 RealSense ROS 2 wrapper(`realsense2_camera`)가 발행하는 토픽을 인지 노드가 직접 구독합니다.
**실제 토픽 이름은 wrapper 버전·설정에 따라 다르므로 추측해서 적지 않고**, 아래처럼 확인한 값을 `config/camera.yaml`(패키지 `ros2_ws/src/realsense_tracker/config/`)에 기록합니다.
(`color_topic`이 비어 있으면 인지 노드는 시작하지 않고 이 안내를 출력합니다.)

```bash
# 1) wrapper 설치 (한 번만, Ubuntu 24.04 + ROS 2 Lyrical)
sudo apt install ros-lyrical-realsense2-camera

# 2) wrapper 실행 — 깊이를 컬러에 정렬(align)해서 발행
#    (파라미터 이름은 wrapper 버전에 따라 다를 수 있음 → ros2 launch realsense2_camera rs_launch.py --show-args 로 확인)
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# 3) 다른 터미널에서 실제 토픽 이름·형식 확인
ros2 topic list | grep -E "color|depth|camera_info"
ros2 topic type <확인한 토픽>
ros2 topic echo <컬러 토픽> --once --field encoding     # bgr8 / rgb8
ros2 topic echo <컬러 토픽> --once --field header        # frame_id, stamp
ros2 topic hz <컬러 토픽>                                # 실제 fps
```

확인한 값을 `config/camera.yaml`(패키지 `ros2_ws/src/realsense_tracker/config/`)의 `color_topic`, `aligned_depth_topic`, `camera_info_topic`에 기록합니다.
가이드에 따라 encoding, frame_id, stamp, 실제 profile, D435 serial·firmware, wrapper 버전, USB 속도도 함께 기록해 두세요.

## 3. 인지 노드 실행 (`perception_node`)

```bash
# Raspberry Pi, 워크스페이스 빌드·source 후 (../README.md 8절)
ros2 launch realsense_tracker tracker.launch.py          # wrapper + perception (camera_config 자동)
# 또는 인지만 단독 실행 (아래 명령들의 $CFG도 이 값)
CFG=$(ros2 pkg prefix realsense_tracker)/share/realsense_tracker/config
ros2 run realsense_tracker perception_node --ros-args --params-file $CFG/tracker.yaml -p camera_config:=$CFG/camera.yaml
```

- 구독: `camera.yaml`의 `color_topic` (bgr8 / rgb8), `use_depth: true`이면 `aligned_depth_topic` (컬러에 정렬된 16UC1 / 32FC1)
  깊이는 별도로 받아 시각이 50 ms 이내인 컬러 영상에만 붙입니다. 깊이가 없어도 컬러 처리·`/target` 발행은 계속됩니다.
- 깊이 거리 gate(13~100 cm)는 `enforce_depth_range: true`일 때만 동작합니다 (기본 false — 깊이는 기록용). launch는 camera_config 경로를 자동으로 넘깁니다.
- 확인 화면: `-p publish_debug_image:=true`로 실행하면 `/perception/debug_image`에 검출 결과를 그린 영상이 나옵니다.
  (`ros2 run rqt_image_view rqt_image_view`로 확인)
- 필요한 패키지: `rclpy`, `sensor_msgs`, `geometry_msgs`, `message_filters`, `python3-numpy`, `python3-opencv`, `python3-yaml`
  (`cv_bridge`는 쓰지 않고 영상을 직접 변환)

### QoS (`target_reliable`, `image_reliable`, 모두 depth 1)

| 대상 | 설정 | 근거 |
|---|---|---|
| `/target` 발행 | **best effort, depth 1** (`target_reliable: false`) | 발제문 규약 "best-effort, depth 1부터 적용" |
| 영상 구독 (컬러·정렬 깊이) | **reliable, depth 1** (`image_reliable: true`) | 아래 실측 — best effort로는 영상이 거의 전달되지 않음 |

영상 구독 실측 (2026-10-06, RealSense wrapper 640×480 30fps, wrapper 발행 QoS RELIABLE·KEEP_LAST 1, Fast DDS 기본 설정, 다른 프로세스에서 6초 구독):

| 구독 QoS | 컬러 (USB 2.1 / USB 3.2) | 정렬 깊이 (USB 2.1 / USB 3.2) |
|---|---|---|
| best effort | 1.2 / 1.5 fps | 0 / 0.2 fps |
| reliable | 29.4 / 29.6 fps | 29.4 / 29.5 fps |

- **best effort 발행은 reliable 구독과 연결되지 않습니다.** `/target`을 받는 쪽(제어 노드 등)은 best effort로 구독해야 합니다.
  (`ros2 topic echo`와 `ros2 bag record`는 발행 쪽 QoS에 맞춰 자동으로 받음)
- 확인 방법: `ros2 topic info -v /target`의 Reliability

### 파라미터 (`tracker.yaml`의 `perception_node`)

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `camera_config` | (빈 값) | `config/camera.yaml`(패키지 `ros2_ws/src/realsense_tracker/config/`) 경로 — 카메라 토픽 이름을 여기서 읽음 (반드시 지정) |
| `hsv_lower`, `hsv_upper` | `[93, 120, 35]`, `[130, 255, 255]` | 목표 색 HSV 범위 (OpenCV: H 0~179) |
| `min_area_ratio` | 0.002 | 화면 넓이 대비 최소 크기 |
| `blur_ksize`, `morph_ksize` | 5, 5 | 블러·잡음 제거 크기 |
| `use_depth` | false (tracker.yaml: true) | 깊이도 구독해 거리(dist)·유효 거리 측정 비율을 확인 화면·로그에 표시 |
| `enforce_depth_range` | false | true일 때만 `depth_min/max_distance_cm` 밖·측정 불가를 미검출로 처리 (선택 기능) |
| `depth_unit_m` | 0.001 | 16UC1 깊이 값 1의 길이(m) |
| `depth_min_valid_ratio`, `depth_max_spread_cm` | 0.5, 5.0 | 이 기준을 못 넘으면 거리를 믿을 수 없다고 보고 비움 |
| `publish_debug_image` | false | 확인 화면 발행 (평가 도구 `eval_frames.py`에 필요) |
| `stats_period_sec` | 5.0 (tracker.yaml: 1.0) | 이 간격(초)마다 처리 FPS·처리 시간 로그 (0이면 끔) |
| `target_reliable` | false | `/target` 발행 QoS (false = best effort, 위 설명) |
| `image_reliable` | true | 영상 구독 QoS (true = reliable, 위 설명) |

## 4. 도구 (HSV 튜너·장면 캡처)

RealSense를 직접 여는 방식(`hsv_tuning.py`, `image_capture.py` 기본값)은 wrapper가 켜져 있으면 쓸 수 없습니다. (카메라는 한 프로그램만 사용 가능)
`image_capture.py --source ros`는 wrapper 토픽에서 받으므로 wrapper를 켜 둔 채로 씁니다.

### 설치 (한 번만)

```bash
cd lv2_module5
python3 -m venv .venv && source .venv/bin/activate
pip install -r tools/requirements.txt
```

### 4-1. HSV 범위 조절 (`hsv_tuning.py`)

```bash
python tools/hsv_tuning.py
```

목표만 흰색으로 깔끔하게 보이도록 트랙바를 맞춘 뒤 `s`로 저장, `q`로 종료합니다.
`tracker.yaml`의 `hsv_lower`, `hsv_upper`, `min_area_ratio`만 바뀌고 주석과 다른 항목은 그대로 유지됩니다.

| 트랙바 | 뜻 | 조절 요령 |
|---|---|---|
| H 최소 / 최대 | 색상 범위 (파랑 ≈ 100~130) | 비슷한 색 물체가 잡히면 범위를 좁힘 |
| S 최소 / 최대 | 채도 (0 = 흰색·회색) | 흰 배경이 잡히면 S 최소를 올림 |
| V 최소 / 최대 | 밝기 (0 = 검정) | 그림자가 잡히면 V 최소를 올림 |
| 최소 크기 | 화면 넓이 × (값 ÷ 10000) | 작은 잡티가 잡히면 올림 |

트랙바 이름이 안 보이면 한글 글꼴(`fonts-noto-cjk`)이 설치되어 있는지 확인하세요.

### 4-2. 장면별 검출 결과 저장 (`image_capture.py`)

```bash
# 권장: RealSense ROS wrapper 토픽에서 받기 (실제 파이프라인과 같은 입력, ROS 환경의 시스템 파이썬, wrapper 실행 중)
python3 tools/image_capture.py --source ros
# 또는: RealSense를 직접 열기 (도구용 .venv, wrapper가 꺼져 있어야 함)
python tools/image_capture.py
# 화면 없이 (SSH 등): 뒤에 --headless 를 붙이고 터미널에 n/e/o/q 입력 후 Enter
```

- 저장 결과는 같은 `detect()`와 `tracker.yaml` 설정으로 계산하므로, 같은 영상이면 인지 노드의 `/target` 값과 같습니다.

| 키 | 장면 |
|---|---|
| `n` | normal — 목표가 잘 보이는 상태 |
| `e` | empty — 목표를 치운 상태 (`found=False`가 정상) |
| `o` | occluded — 목표 일부를 가린 상태 |
| `q` | 종료 |

저장 위치:
- `results/images/detection/<장면>_<시각>_{raw,mask,det}.png` (원본 / 색 마스크 / 검출 결과)
- `results/logs/perception/log.csv` — 열: `time, scene, found, ex, ey, z, dist_cm, n_candidates, width, height, hsv_ranges, min_area_ratio`
- 같은 초에 같은 장면을 다시 저장하면 이름 뒤에 `_2`, `_3`이 붙습니다.

## 5. 성능 평가 (발제문 문제 4 — 처리 FPS, 검출률, 배경 오검출)

### 5-1. 처리 FPS

인지 노드가 `stats_period_sec`(기본 5초)마다 로그를 남깁니다. 발제문 산식(처리 완료 프레임 수 ÷ 실제 경과 초)이며 카메라 설정 FPS와 다릅니다.

```text
처리 FPS 30.0 (최근 5.0초 동안 150장 처리), 처리 시간 평균 2.8 ms·최대 3.4 ms, 노드가 검출로 표시한 프레임 150/150 (사람 대조 검출률과 다름)
```

- 처리 시간 = 영상 변환부터 `/target` 발행까지 (노드 안에서 걸린 시간). 촬영→구동 지연과는 다릅니다.
- 로그는 터미널과 `~/.ros/log/`에 남습니다. 정상 추적 30초 시험 동안의 로그를 `results/logs/perception/`에 보관하세요.

### 5-2. 검출률·배경 오검출 (사람 대조)

발제문 기준: 목표가 보이는 프레임을 **고르게 최소 30장**, 목표가 없는 프레임을 **최소 10장** 골라 사람이 대조하고 목록을 저장합니다.
노드가 스스로 검출로 표시한 비율은 정답 대조 검출률과 다릅니다.

```bash
# 모든 터미널에서 같은 ROS_DOMAIN_ID 사용 (같은 네트워크의 다른 팀·장비 노드와 섞이지 않도록) 예: export ROS_DOMAIN_ID=42
# 터미널 1: RealSense wrapper (2장 참고)
# 터미널 2: 인지 노드 — 확인 화면 발행 필요
ros2 run realsense_tracker perception_node --ros-args \
  --params-file $CFG/tracker.yaml \
  -p camera_config:=$CFG/camera.yaml -p publish_debug_image:=true
# 터미널 3 (lv2_module5 폴더, ROS 환경): 평가 프레임 저장
python3 tools/eval_frames.py --scene visible --note "거리 40cm, 실내 조명"   # SPACE 후 15초 동안 30장
python3 tools/eval_frames.py --scene empty   --note "목표 치움"              # SPACE 후 10초 동안 10장
```

- 실행하면 창에 인지 노드의 검출 화면이 실시간으로 뜹니다. 장면을 준비한 뒤 창을 클릭하고 **SPACE**를 누르면 저장을 시작하고,
  창 아래에 `REC 저장한 장수/전체`가 표시됩니다. **q**(또는 ESC)는 종료합니다. (화면 없이 바로 저장: `--no-view`)
- visible은 저장하는 동안 목표를 화면 안의 여러 위치(좌·우·위·아래·중앙)로 천천히 옮기면 "고르게 고른" 프레임이 됩니다.

저장 결과:
- `results/images/evaluation/<scene>_<시각>/NN_raw.png`, `NN_det.png` — 원본 / 노드 검출 결과 그림
- `results/logs/perception/eval_<scene>_<시각>.csv` — 프레임마다 시각·노드 출력(`node_detected, ex, ey, area_ratio`)과 판정 칸
- `results/logs/perception/eval_runs.csv` — 실행 기록 (장면·장수·기간·`/target` 수신 Hz·HSV·최소 면적·카메라·메모)

판정과 집계:
1. CSV를 열고 각 `NN_det.png`를 보며 `human_ok` 칸에 **1(노드 출력이 맞음) / 0(틀림)** 을 적습니다. 필요하면 `note`에 이유를 적습니다.
   - visible: 목표에 외곽선·중심이 맞게 그려졌으면 1, 놓쳤거나(미검출) 다른 물체를 잡았으면 0
   - empty: 아무것도 검출하지 않았으면 1, 무언가를 검출했으면 0 (배경 오검출)
2. 집계:
   ```bash
   python3 tools/eval_score.py results/logs/perception/eval_visible_*.csv results/logs/perception/eval_empty_*.csv
   ```
   ```text
   검출률 = 올바른 검출 / 판정한 프레임 × 100   (틀린 프레임은 미검출 / 다른 물체 검출로 구분해 출력)
   배경 오검출 = 틀린 프레임 수 (검출률과 별도)
   ```

## 6. 알아둘 점

- **목표는 가장 큰 덩어리 1개만 고릅니다.** 비슷한 색의 더 큰 물체(남색, 하늘색 등)가 있으면 그쪽을 목표로 잡을 수 있습니다.
  화면의 `candidates`가 2 이상이면 다른 후보가 있다는 뜻입니다.
- **`z`는 거리가 아닙니다.** 거리의 제곱에 반비례하고, 목표가 가려지면 작아집니다.
- **거리(dist_cm) 실측 결과** (D435, 640×480):
  - 가린 것이 없으면 약 17cm ~ 68cm에서 오차 3% 이내 (17cm보다 가까운 거리는 측정하지 않음)
  - 목표 바로 앞을 손으로 가려도 거리는 정확
  - 손 같은 물체가 **카메라 가까이**에서 가리면 거리를 잴 수 없음 (적외선 카메라 두 대 중 한쪽 시야만 가려짐)
    → 틀린 값 대신 `--`(빈칸)가 나오도록 `depth_*` 두 기준으로 걸러냄
  - 화면 왼쪽 끝 부근은 RealSense 특성상 깊이가 측정되지 않아 `--`가 나올 수 있음
