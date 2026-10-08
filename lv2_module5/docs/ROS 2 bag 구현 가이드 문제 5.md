# ROS 2 bag 구현 가이드 — 문제 5 (bag 재현과 팀 협업)

> 대상: 통합 담당(기록·재생 구성), 검증 담당(지표 비교), 재현 확인을 맡은 팀원.
> 기준: 발제 문서 문제 5 · 6장 산출물 4 · 7장 평가표 9번, 코드 기준 commit `be2cf59` (브랜치 `dev/test1`).
> 이 프로젝트에서는 아래 절차를 스크립트로 자동화했다 — **기록 `tools/bag_record.sh`, 재현 `tools/bag_replay.sh`, 확인·대조 `tools/bag_tool.py`, 토픽 설정 `config/bag.yaml`**.
> 각 절의 "스크립트" 상자가 실제로 쓸 명령이고, 그 아래 수동 명령은 스크립트가 하는 일의 설명이자 스크립트 없이 할 때의 방법이다.
> 실행 명령 요약은 [README 18~20절](../README.md#18-bag-record), 결과 정리 표는 [recordings/README.md](../recordings/README.md)에 있다. 이 가이드는 **왜·어떤 순서로·무엇을 확인하며** 할지 풀어 쓴 것이다.

목차: [1 요구사항](#1-발제-요구사항과-남길-증거) · [2 흐름](#2-전체-흐름) · [3 준비](#3-준비) · [4 기록](#4-기록-raspberry-pi-실제-추적-중) · [5 메타데이터](#5-메타데이터체크섬업로드) · [6 재생 안전](#6-재생-전-안전-모터-출력-끄기) · [7 A 재처리](#7-a-입력-재처리) · [8 B 재분석](#8-b-결과-재분석) · [9 비교](#9-결과-비교와-판정) · [10 팀원 재현](#10-작성자가-아닌-팀원의-재현) · [11 보고서](#11-reportmd--teammd-작성) · [12 문제 해결](#12-자주-나는-문제) · [13 주의점](#13-현재-도구-기준-주의점) · [14 도전 E](#14-선택-도전-e--고정-bag-회귀-비교)

---

## 1. 발제 요구사항과 남길 증거

| # | 발제 요구 | 이 가이드 절 | 남길 증거 (위치) |
|---|---|---|---|
| 1 | 대표 **성공** 장면과 **소실·복귀** 장면을 각각 10~30 s 기록, 용량 확인 | 4 | bag 2개 (외부 저장소), `ros2 bag info` 출력 |
| 2 | 영상·목표·상태·명령 토픽 기록, 시리얼 로그는 **같은 실행 ID**로 연결 | 4.2, 4.1 | bag, `<RUN>_serial.csv`, `<RUN>.csv` |
| 3 | 토픽·메시지 수·기간·해상도·설정·기준 커밋 기록, **metadata.yaml과 데이터 파일 함께 보관** | 5 | `recordings/README.md` 3절 표, `SHA256SUMS` |
| 4 | 재현 중 **실제 모터 출력 비활성화** | 6 | 재현 기록에 확인 항목 |
| 5 | **입력 재처리**: bag 영상만 검출기로, `/target_replay` 등 별도 출력 | 7 | `<RUN>_reprocess.csv`, 재처리 이미지 |
| 6 | **결과 재분석**: 저장된 목표·상태·명령으로 지표 재계산 | 8 | `<RUN>_reanalysis.csv`, 재계산 지표 |
| 7 | 저장된 `/target`과 새 검출 결과를 같은 토픽에 섞지 않음, 실제 remap 명령을 README에 | 7 | README 19절 (이미 있음) |
| 8 | `--clock` + `use_sim_time`, 과거 bag 시각과 현재 벽시계를 섞지 않음 | 7, 8, 13 | 실행 명령 |
| 9 | 오프라인 재현과 실제 하드웨어 폐루프 시연 구분 | 11 | report.md 문제 5 |
| 10 | **작성자가 아닌 팀원**이 README만 보고 실행, 확인자·날짜·기준 커밋·결과 기록, 누락 수정 | 10 | `recordings/README.md` 4절, 수정 PR |
| 11 | 입력 재처리와 결과 재분석의 **차이 설명** | 9, 11 | report.md 문제 5 |

평가표 9번 판정 문구: *"파일 누락 없이 재생되고 입력 재처리와 출력 재분석의 차이를 설명하는가."*

---

## 2. 전체 흐름

```text
[Pi · 실제 추적]                                [외부 저장소]                 [다른 팀원 · 모터 OFF]
 opencr_node (csv_path=<RUN>_serial.csv) ─┐
 tracker.launch (camera+perception+control)├─▶ ros2 bag record ─▶ bag ─▶ sha256 ─▶ 업로드 ─▶ 다운로드·sha256 -c
 tracking_logger --run-id <RUN>  ─────────┘   (<RUN>.csv 실시간)                              │
                                                                                         ┌───────┴────────┐
                                                                                  A. 입력 재처리     B. 결과 재분석
                                                                               영상만 → perception   /target·상태·명령
                                                                               → /target_replay      → tracking_logger
                                                                               → <RUN>_reprocess.csv → <RUN>_reanalysis.csv
                                                                                         └───────┬────────┘
                                                                                     analyze_tracking + 프레임 대조
                                                                                                 ▼
                                                                                   report.md 문제 5 · recordings 표
```

한 실행(run)마다 같은 `RUN` 이름으로 아래 파일이 생긴다.

| 파일 | 만드는 것 | 저장 위치 | Git |
|---|---|---|---|
| `~/bags/<RUN>/` (`metadata.yaml` + `*.mcap` 또는 `*.db3`) | `ros2 bag record` | Pi → 외부 저장소 | ✗ (`.gitignore`) |
| `<RUN>.csv` | `tracking_logger` (실시간) | `results/logs/verification/` | ✓ |
| `<RUN>_serial.csv` | `opencr_node csv_path` | Pi `~/runs/` → `results/logs/verification/`로 복사 | ✓ |
| `<RUN>_reanalysis.csv` | `tracking_logger` (B) | `results/logs/verification/` | ✓ |
| `<RUN>_reprocess.csv` | `tracking_logger --target-topic /target_replay` (A) | `results/logs/verification/` | ✓ |
| `<RUN>_serial.csv`와 같은 실행의 bag 증거: `source.txt`(commit·환경), `info.txt`, `check.txt`, `SHA256SUMS`, `ARCHIVE.sha256`, `params/`(실제 파라미터), `config/` | `bag_record.sh` | `recordings/<RUN>/` | ✓ |
| 재현 실행 기록: `run.txt`, 노드 로그, `analyze.txt`, `compare.txt` | `bag_replay.sh` | `results/logs/replay/<RUN>_<mode>_<시각>/` | ✓ |
| 재처리 이미지 | `bag_replay.sh reprocess --images N` | `results/images/replay/<RUN>_reprocess<suffix>/` | ✓ |

실행 ID 규칙: 성공 `success_01`, 소실·복귀 `lost_01`. 다시 찍으면 번호를 올린다 (`success_02`). **실패한 기록도 지우지 않고** 번호를 남긴다.

---

## 3. 준비

### 3.1 역할 분담 (권장)

| 단계 | 담당 | 비고 |
|---|---|---|
| 기록 (4절) | 통합 + 검증 | 실제 추적 중이라 **카메라를 지지할 사람 1명** 별도 |
| 메타데이터·업로드 (5절) | 통합 | |
| 재생 A·B, 비교 (6~9절) | **기록하지 않은 팀원** | 발제: "작성자가 아닌 팀원이 README만 보고 실행" |
| 보고서·team.md (11절) | 검증·문서화 | |

시작 전에 Issue를 만들어 담당자와 완료 조건을 적는다. 예: *"success_01·lost_01 bag 기록, 다른 팀원이 A·B 재현, report 문제 5 작성"*.

### 3.2 환경 (Pi)

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/Lv2_EYE4_Assignment/lv2_module5/ros2_ws/install/setup.bash   # PanTiltCommand 타입 — 없으면 명령 토픽 기록·재생 불가
cd ~/git/Lv2_EYE4_Assignment/lv2_module5
CFG=$PWD/ros2_ws/src/realsense_tracker/config
ros2 bag record --help | head -40     # 이 Pi의 rosbag2 옵션 확인 (--topics, -s, --max-cache-size 지원 여부)
```

`realsense_tracker_interfaces`가 빌드·source되지 않은 셸에서는 `/control/pan_tilt_cmd`의 타입을 몰라 기록·재생에서 빠진다. **기록·재생하는 모든 터미널**에서 워크스페이스를 source한다.

### 3.3 저장 공간과 부하

| 토픽 | 크기 계산 | 10 s | 30 s |
|---|---|---|---|
| `/camera/camera/color/image_raw` 640×480 rgb8 30 fps | 921,600 B × 30 ≈ 27.6 MB/s | ≈ 0.28 GB | ≈ 0.83 GB |
| 나머지 (`/target`, 상태, 명령, bridge_status, camera_info) | 수십 KB/s | 무시 가능 | 무시 가능 |

```bash
df -h ~                                  # 여유 공간: 기록 예정량의 3배 이상
```

- microSD는 연속 쓰기 속도가 27.6 MB/s보다 느릴 수 있다. 그러면 메시지가 빠진다(4.4절에서 확인). 빠지면 **USB SSD에 기록**한다 (`-o /media/<ssd>/bags/$RUN`).
- 기록 자체가 Pi CPU를 쓴다. 기록 중 `perception_node`의 "처리 FPS" 로그가 기록 전보다 떨어지는지 보고, 떨어지면 report에 적는다(발제: 측정 조건 변화 기록).
- 압축(`--compression-mode`)은 CPU를 더 써서 추적에 영향을 줄 수 있다. **기본은 비압축**, 업로드 전에 따로 압축(5.3절)한다.
- 정렬 깊이 영상은 **기록하지 않는다**. 현재 검출은 컬러만 쓰고(`enforce_depth_range: false`), 깊이까지 넣으면 용량이 약 1.7배가 된다.

### 3.4 SSH 끊김 대비

SSH가 끊기면 `ros2 bag record`가 정상 종료되지 못해 `metadata.yaml`이 없는 bag이 남을 수 있다. 기록 터미널은 `tmux`(또는 `screen`) 안에서 실행한다.

```bash
tmux new -s bag            # 끊기면 다시 ssh 후: tmux attach -t bag
```

---

## 4. 기록 (Raspberry Pi, 실제 추적 중)

### 4.1 터미널 구성

사전조건은 [README 14절](../README.md#14-실제-tracking-실행) 그대로다. LIVE 펌웨어, 중립 자세, 카메라 지지.

```bash
RUN=success_01
mkdir -p ~/runs ~/bags

# T1: OpenCR bridge — 시리얼 로그를 같은 RUN으로 (같은 파일이 있으면 시작 거부: 증거 보호)
ros2 run realsense_tracker opencr_node --ros-args --params-file $CFG/opencr_live.yaml \
  -p csv_path:=$HOME/runs/${RUN}_serial.csv

# T2: 카메라 + 인지 + 제어
ros2 launch realsense_tracker tracker.launch.py start_control:=true

# T3: prepare → (TRACKING 확인 후) arm   — README 14절
ros2 service call /opencr/prepare std_srvs/srv/Trigger
ros2 service call /opencr/arm std_srvs/srv/Trigger

# T4: 실시간 추적 CSV
python3 tools/tracking_logger.py --run-id $RUN

# T5 (tmux): bag 기록 — 4.2절
tools/bag_record.sh $RUN --duration 20 --archive
```

**순서**: T1~T3로 추적이 안정된 것을 확인 → T4 시작 → T5 시작 → 시나리오 수행(4.3절) → **T5 Ctrl+C → T4 Ctrl+C**. 정지는 README 15절 순서로 한다.

### 4.2 기록 명령

> **스크립트**: `tools/bag_record.sh <RUN> [--duration SEC] [--archive] [--out-dir DIR] [--force]`
> 1) ROS·워크스페이스 source, 같은 RUN 없음, bag 폴더가 Git 저장소 밖, 저장 공간(예상 × 2), **필수 토픽이 지금 발행 중인지** 확인
> 2) perception·control·opencr 노드의 실제 파라미터를 `ros2 param dump`로 `recordings/<RUN>/params/`에 저장 (실행 시 `-p`로 바꾼 값까지)
> 3) 아래 토픽으로 `ros2 bag record` — Ctrl+C 또는 `--duration` 만료 때 SIGINT를 **한 번만** 전달해 정상 종료
> 4) `ros2 bag info`, 4.4절 판정(`bag_tool check`), 파일별 sha256, 기준 commit을 `recordings/<RUN>/`에 저장
> 기록 토픽은 [config/bag.yaml](../config/bag.yaml)에 있고, 카메라 토픽 이름은 camera.yaml에서 읽는다.

수동으로 같은 기록:

```bash
ros2 bag record -o ~/bags/$RUN -s mcap --topics $(python3 tools/bag_tool.py topics record)
```

- 토픽은 `--topics`로 넘긴다(새 rosbag2는 위치 인자에 경고). 스크립트는 `--help`를 보고 지원하지 않으면 위치 인자로 넘긴다.
- `-s mcap`: Jazzy 이후 기본 저장 형식이다. 어느 형식이든 `.gitignore`가 막는다. 바꾸려면 `BAG_STORAGE=sqlite3`.

| 토픽 | 타입 | 주기(정상) | 왜 기록하나 |
|---|---|---|---|
| `/camera/camera/color/image_raw` | sensor_msgs/Image (rgb8) | 30 Hz | **A 입력 재처리**의 입력 |
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | 30 Hz | 해상도·내부 파라미터 증거 (메타데이터 "해상도") |
| `/target` | geometry_msgs/PointStamped | 영상당 1회 (≈30 Hz) | **B 재분석** 입력, A 비교 기준 |
| `/tracking_status` | std_msgs/String | 20 Hz + LOST 전환 | B: 상태(IDLE/TRACKING/LOST) |
| `/control/pan_tilt_cmd` | realsense_tracker_interfaces/PanTiltCommand | 20 Hz | B: 명령값 |
| `/opencr/bridge_status` | std_msgs/String (JSON) | 100 Hz | bridge phase·FAULT 원인·보드 STATE (안전 동작 증거) |

선택: `/rosout`도 넣으면 노드 경고·FAULT 로그가 같은 시간축에 남는다(용량 작음).

### 4.3 시나리오 (시험 전에 확정하고 Issue에 기록)

**success_NN — 대표 성공 장면 (목표 15~25 s)**

| 시각 | 동작 |
|---|---|
| 0~3 s | 목표 중앙, TRACKING 유지 |
| 3~9 s | 목표를 천천히 왼쪽 → 오른쪽으로 이동 |
| 9~15 s | 위·아래로 이동 |
| 15~18 s | 중앙 복귀 |

**lost_NN — 소실·복귀 장면 (목표 15~25 s)**

| 시각 | 동작 |
|---|---|
| 0~4 s | TRACKING 안정 |
| 4~6 s | 손으로 **약 2 s 가림** → LOST, 정지 |
| 6~10 s | **시야 안**에서 재등장 → 3프레임 후 TRACKING |
| 10~12 s | 다시 2 s 가림 |
| 12~16 s | 재등장·추적 |

- **이 프로젝트의 제약** (commit `be2cf59` 기준):
  - 속도 상한 0.10 rad/s(약 5.7°/s, `control.yaml`·펌웨어 `MAX_RAD_S`) → 목표를 그보다 천천히 움직여야 따라간다. 빨리 움직이면 시야를 벗어나 LOST가 된다.
  - 펌웨어 경계는 CHECK 때 자세 기준 Pan ±1345 / Tilt ±662 counts(약 ±118° / ±58°). 기구 한계 승인값이 아니므로 시나리오는 그보다 훨씬 안쪽에서 한다.
  - 펌웨어는 경계에서 `EVENT LIMIT STOPPED`(ARM 유지)를 보내지만, 현재 bridge(`serial_core.py`)는 `EVENT LIMIT`으로 시작하는 줄을 FAULT로 처리한다 → 경계에 닿으면 세션이 끝나고 bag에 FAULT가 남는다. 기록 중에는 경계에 닿지 않게 한다.
- 문제 4의 "2 s 가림 5회"는 `tracking_logger` CSV와 `recovery_trials.csv`로 따로 기록한다. bag에 5회를 모두 넣으려면 30 s를 넘기 쉽다(≈0.8 GB 이상).
- 가림·재등장 시각을 사람이 판정할 수 있게 휴대폰으로 같은 장면을 촬영해 두면 좋다(영상 파일은 bag과 같은 외부 저장소에 같은 RUN 이름으로).

### 4.4 기록 직후 확인 (Pi, 같은 자리에서)

> **스크립트**: `bag_record.sh`가 끝나면서 자동으로 하고 `recordings/<RUN>/info.txt`, `check.txt`에 남긴다. 아래 표 기준(`config/bag.yaml`의 `rate_hz × min_rate_ratio`)으로 토픽마다 OK/WARN/FAIL을 찍고,
> 파일 누락·필수 토픽 없음이면 FAIL(exit 1). 다시 확인만 하려면 `python3 tools/bag_tool.py check ~/bags/$RUN`.

```bash
ros2 bag info ~/bags/$RUN | tee ~/runs/${RUN}_info.txt
du -sh ~/bags/$RUN
ls -l ~/bags/$RUN                    # metadata.yaml + 데이터 파일(들)이 모두 있는지
```

메시지 수가 기대와 맞는지 본다 (기간 D초):

| 토픽 | 기대 개수 | 크게 모자라면 |
|---|---|---|
| image_raw | ≈ 30 × D | 저장 속도 부족 → SSD에 다시 기록 |
| `/target` | ≈ image_raw 개수 (인지 처리 FPS에 따라 약간 적을 수 있음) | 인지가 느림 → 처리 FPS 로그 확인 |
| `/tracking_status`, `/control/pan_tilt_cmd` | ≈ 20 × D (+ LOST 전환) | 제어 타이머 지연 |
| `/opencr/bridge_status` | ≈ 100 × D | |

**기간이 10 s 미만이거나 30 s를 크게 넘으면** 다시 기록한다. 원래 기록은 지우지 말고 번호만 올린다.

---

## 5. 메타데이터·체크섬·업로드

### 5.1 체크섬은 파일 단위로

> **스크립트**: `bag_record.sh`가 `recordings/<RUN>/SHA256SUMS`(bag 폴더 안 파일별), `--archive`면 `ARCHIVE.sha256`(압축본)을 만든다.

```bash
cd ~/bags/$RUN && sha256sum metadata.yaml *.mcap *.db3 2>/dev/null > <lv2_module5>/recordings/$RUN/SHA256SUMS
```

- `tar -cf - $RUN | sha256sum`처럼 tar 스트림의 체크섬을 쓰면 tar가 파일 시각·소유자를 포함해서, 내려받은 사람이 같은 값을 다시 만들 수 없다. **파일별 `sha256sum`** 을 쓰고 받은 쪽은 `sha256sum -c`로 확인한다(10절).

### 5.2 기록할 메타데이터 (실제 값만)

| 항목 | 얻는 방법 |
|---|---|
| run ID, 목적(성공/소실) | 4.3절 |
| 기간, 토픽별 메시지 수, 저장 형식 | `ros2 bag info` |
| 해상도 | camera_info 또는 `config/camera.yaml` (640×480 rgb8 30 fps) |
| 설정 | `recordings/<RUN>/params/`(실제 실행 값), `config/`(파일 사본) |
| 기준 commit | `source.txt`의 `git_commit` **+** `git_dirty_files` (커밋 안 된 변경이 있으면 "dirty"와 변경 파일을 같이 적음) |
| 펌웨어 | 업로드한 빌드 로그 폴더 (`results/logs/opencr/...`) |
| 크기·sha256 | 5.1절 |
| 시리얼 로그 | `<RUN>_serial.csv` 경로 |
| 위치(링크) | 5.3절 |

설정이 commit과 다르면(실행 중 `-p kp_pan:=...`로 바꾼 경우 등) 실제 사용 값을 따로 적는다. T2 launch 로그 첫 줄의 `control_node: kp_pan=...` 출력이 근거다.

### 5.3 업로드와 Git 커밋

```bash
cd ~/bags && tar -czf ${RUN}.tar.gz $RUN && sha256sum ${RUN}.tar.gz > <lv2_module5>/recordings/$RUN/ARCHIVE.sha256   # = --archive
```

- 압축본을 팀 공유 드라이브에 올리고, **평가자 계정으로 접근 가능한지** 다른 팀원이 확인한다(발제 6장: "링크 접근과 다운로드를 제출 전에 다른 팀원이 확인").
- Git에는 bag을 넣지 않는다. 대신 다음을 PR로 올린다:
  - `results/logs/verification/<RUN>.csv`, `<RUN>_serial.csv`
  - `recordings/<RUN>/` 폴더 전체 (source.txt, info.txt, check.txt, SHA256SUMS, ARCHIVE.sha256, params/, config/)
  - `recordings/README.md` 3절 표에 1행 추가 (5.2절 항목, 링크)
- 개인 PC 경로(`/home/<이름>/...`)만 적지 않는다. 링크와 파일명을 적는다.

---

## 6. 재생 전 안전 (모터 출력 끄기)

발제: *"bag 재생 중 기록된 명령을 실제 모터에 재전송하지 않습니다."* 아래 세 겹을 **모두** 적용하고, 재현 기록에 체크한다.

| 겹 | 방법 | 확인 |
|---|---|---|
| 1 물리 | OpenCR USB를 뽑거나 모터 전원 OFF (카메라 지지 후 — Tilt 낙하 주의) | `ls /dev/serial/by-id/`에 OpenCR 없음 |
| 2 프로세스 | `pkill -f opencr_node; pkill -f control_node` | `ros2 node list`에 둘 다 없음 |
| 3 네트워크 | 모든 재생 터미널에서 `export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | 실시간 추적에 쓰던 도메인과 다름 |

- 위 세 겹이 모두 실패해도, bridge는 `header.stamp`가 0.15 s(`opencr_live.yaml command_max_age_sec`)보다 오래된 명령을 거부한다(bag의 명령은 과거 시각). 다만 이것은 **마지막 방어선**이지 계획한 차단이 아니다.
- **스크립트**: `bag_replay.sh`는 시작 전에 ② opencr_node·control_node 프로세스, ① `/dev/serial/by-id/*OpenCR*` 연결(모터 전원을 끈 경우에만 `--motor-off-confirmed`), ③ 격리 도메인에 다른 노드가 있는지를 확인하고, 하나라도 걸리면 거부한다. 확인 결과는 `run.txt`의 `motor_output`에 남는다.
- 모든 재생 터미널에서 같은 `ROS_DOMAIN_ID`를 쓴다. 다르면 노드끼리 서로 보이지 않는다.
- 재생은 Pi 또는 ROS 2가 설치된 팀원 노트북에서 한다. 노트북에서는 **워크스페이스를 빌드·source**해야 `PanTiltCommand`를 재생할 수 있다.

---

## 7. A. 입력 재처리

**목적**: bag의 **원본 영상만** 현재 검출기에 다시 넣었을 때 같은 설정에서 같은 검출·오차가 나오는지 확인한다. 저장된 `/target`을 보기만 하는 것은 재처리가 아니다.

> **스크립트**: `tools/bag_replay.sh reprocess ~/bags/<RUN> [--images N] [--rate R] [--suffix S] [--current-config]`
> 6절 안전 확인 → 아래 T1·T2를 백그라운드로, T3를 앞에서 실행 → 재생이 끝나면 T2·T1을 SIGINT로 정상 종료(CSV 저장) →
> `analyze_tracking` → `<RUN>_reanalysis.csv`(없으면 `<RUN>.csv`)와 9절 대조. 모든 명령·로그는 `results/logs/replay/<RUN>_reprocess_<시각>/`.
> 검출기 파라미터는 **기록 당시 실제 값** `recordings/<RUN>/params/perception_node.yaml`을 쓴다(없으면 경고 후 현재 tracker.yaml, `--current-config`로 강제).

수동으로 같은 재처리:

```bash
RUN=success_01; BAG=~/bags/$RUN          # 다운로드했다면 압축을 풀고 sha256sum -c 먼저 (10절)
PARAMS=recordings/$RUN/params/perception_node.yaml   # 기록 당시 실제 값 (없으면 $CFG/tracker.yaml)

# T1: 검출기 — 출력을 /target_replay로 분리, 깊이 없음(bag에 없음), bag 시계 사용
ros2 run realsense_tracker perception_node --ros-args --params-file $PARAMS \
  -p camera_config:=$CFG/camera.yaml -p use_depth:=false -p use_sim_time:=true \
  -p publish_debug_image:=true -r /target:=/target_replay

# T2: 재처리 결과 기록
python3 tools/tracking_logger.py --run-id ${RUN}_reprocess --target-topic /target_replay \
  --ros-args -p use_sim_time:=true

# T3: 영상만 재생 (T1·T2가 뜬 뒤)
ros2 bag play $BAG --clock --delay 2 --rate 0.5 --topics $(python3 tools/bag_tool.py topics reprocess)
# 재생이 끝나면 T2 Ctrl+C → "N행 저장" 확인, T1 Ctrl+C
```

| 옵션 | 이유 |
|---|---|
| `-r /target:=/target_replay` | 저장된 `/target`과 새 결과를 섞지 않음 (발제 필수) |
| `--params-file` 기록 당시 dump | 발제 "같은 설정에서" 재현. 현재 tracker.yaml이 그 뒤 바뀌었어도 기록 당시 HSV·면적으로 검출 |
| `-p use_depth:=false` | `tracker.yaml`은 `use_depth: true`인데 bag에 깊이 영상이 없음 |
| `-p use_sim_time:=true` + `--clock` | 노드 시계를 bag 시각에 맞춤 (`tracking_logger`의 `receive_time_s`가 과거 시각 기준이 됨) |
| `--delay 2` | 구독자가 연결되기 전에 첫 프레임이 지나가지 않게 |
| `--rate 0.5` | 검출기가 30 fps를 못 따라가도 프레임이 빠지지 않게 (프레임 단위 대조용). 시각은 bag 시각이라 결과에 영향 없음 |

**재처리 이미지** (발제 결과물 "재처리 이미지"):

> **스크립트**: `tools/bag_replay.sh reprocess ~/bags/<RUN> --images 10 --suffix _img` → `results/images/replay/<RUN>_reprocess_img/`
> 재생 실제 시간(bag 기간 ÷ rate)의 80 % 동안 고르게 N장(원본·검출 그림 + 프레임 목록 CSV).

수동: T1이 떠 있는 상태에서 T2 대신 또는 함께

```bash
python3 tools/eval_frames.py --scene visible --target-topic /target_replay --no-view \
  --frames 10 --duration 15 --output-dir results/images/replay/${RUN}_reprocess_img \
  --note "replay $RUN reprocess (not human-eval set)"
# 그 다음 T3 재생. --duration은 재생 실제 시간(bag 기간 ÷ rate)보다 짧게 (eval_frames는 실제 경과 시간으로 간격을 나눔)
```

`--output-dir`를 주면 이미지·목록이 그 폴더에만 저장되고 `eval_runs.csv`(사람 대조 평가 실행 목록)에는 기록되지 않는다. 문제 1·4의 사람 대조 30프레임 세트와 섞이지 않는다.

---

## 8. B. 결과 재분석

**목적**: 저장된 출력(`/target`, `/tracking_status`, `/control/pan_tilt_cmd`)을 같은 기록기·같은 지표 코드에 다시 넣어, 실시간 성능표와 같은 값이 나오는지 확인한다. 검출기는 실행하지 않는다.

> **스크립트**: `tools/bag_replay.sh reanalysis ~/bags/<RUN>` — 6절 안전 확인 → 아래 T1·T2 → `analyze_tracking` → 실시간 `<RUN>.csv`와 9절 대조(`--state` 포함).

수동으로 같은 재분석:

```bash
# T1
python3 tools/tracking_logger.py --run-id ${RUN}_reanalysis --ros-args -p use_sim_time:=true
# T2 (T1이 뜬 뒤)
ros2 bag play $BAG --clock --delay 2 --topics $(python3 tools/bag_tool.py topics reanalysis)
# 끝나면 T1 Ctrl+C
python3 tools/analyze_tracking.py results/logs/verification/${RUN}.csv \
                                  results/logs/verification/${RUN}_reanalysis.csv --plot
```

- 이 재생에는 control_node·opencr_node가 없으므로 재생된 명령을 받는 쪽은 `tracking_logger`뿐이다(6절이 그래도 필수).
- 지표를 `results/metrics.csv`에 남길 때는 `--append-metrics --problem P5`를 쓴다. 실시간 행(P3/P4)과 구분된다.

---

## 9. 결과 비교와 판정

### 9.1 무엇과 무엇을 비교하나

| 비교 | 기준 CSV | 대상 CSV | 기대 결과 | 다르면 의심할 것 |
|---|---|---|---|---|
| **B ↔ 실시간** | `<RUN>.csv` (기록 구간만) | `<RUN>_reanalysis.csv` | 같은 `stamp_ns`의 ex·ey·area_ratio **완전히 같음**. 프레임 수·FPS·RMSE·유효 추적 비율 거의 같음 | 실시간 기록기가 bag보다 먼저/늦게 시작(구간 차이), best effort 손실, 상태·명령 정렬(9.3절) |
| **A ↔ B** | `<RUN>_reanalysis.csv` (bag의 `/target`) | `<RUN>_reprocess.csv` | 같은 `stamp_ns` 프레임의 detected·ex·ey **완전히 같음** (같은 코드·설정이면 검출기는 결정적) | 재처리 때 설정·코드가 기록 당시와 다름, 프레임 손실, 영상 변환 차이 |

`perception_node`는 출력 `header`에 **입력 영상의 header를 그대로 복사**한다. 그래서 원본 `/target`과 재처리 `/target_replay`는 같은 영상에 대해 같은 `stamp_ns`를 가진다. 이것이 프레임 단위 대조의 열쇠다.

### 9.2 프레임 단위 대조

> **스크립트**: `bag_replay.sh`가 끝에 자동 실행하고 결과를 `compare.txt`에 남긴다. 따로 돌릴 때:

```bash
python3 tools/bag_tool.py compare results/logs/verification/${RUN}.csv            results/logs/verification/${RUN}_reanalysis.csv --state   # B ↔ 실시간
python3 tools/bag_tool.py compare results/logs/verification/${RUN}_reanalysis.csv results/logs/verification/${RUN}_reprocess.csv            # A ↔ B
```

출력: 두 CSV의 **공통 영상 시각 구간**만 대상으로 공통 프레임 수, 구간 안에서 한쪽에만 있는 프레임 수(빠진 프레임), 필드별(detected·ex·ey·area_ratio) 일치 수·최대 차, 불일치 예시.
값이 하나라도 다르면 `결과: FAIL`(exit 1). 빠진 프레임만 있으면 PASS + 개수 경고. `--state`는 state 일치도 보여 주되 판정에서는 뺀다(9.3절) — 단, 10 %를 넘게 다르면 `/tracking_status`가 재생되지 않은 것이므로 경고한다.
허용 차를 두려면 `--tol 1e-5` (기본 0 = 완전히 같음).

### 9.3 차이가 정상인 경우 (보고서에 설명할 것)

- **상태·명령 열**: `tracking_logger`는 `/target`을 받는 순간의 "최신" 상태·명령을 붙인다. 재생에서는 토픽 사이 도착 순서가 실시간과 조금 다를 수 있어, LOST↔TRACKING **전환 직후 1~2프레임**의 state가 다를 수 있다. 그 결과 유효 추적 비율·RMSE가 아주 약간 다를 수 있다.
- **구간**: 실시간 CSV는 기록기를 켠 동안 전체, bag은 `ros2 bag record`를 켠 동안만 담긴다. 비교는 공통 `stamp_ns` 구간으로 한다.
- **A의 지표**: 재처리 CSV에는 상태·명령이 없으므로(13절) 유효 추적 비율 0 %, RMSE `nan`이 나온다. **A에서는 검출 여부와 ex·ey 대조만** 의미가 있다.
- **A의 처리 FPS**: `analyze_tracking`의 FPS는 원본 영상 시각으로 계산한다. 따라서 재처리 FPS는 "빠진 프레임이 없었는가"를 보여 줄 뿐이고, 실시간 처리 성능이 아니다.

### 9.4 판정 기준 (시험 전에 Issue에 확정)

권장 기준: *"A·B 모두 공통 프레임의 검출 여부·ex·ey가 완전히 같다(차 0). 프레임 손실은 개수와 원인을 기록한다."*
기준을 결과를 본 뒤 바꾸지 않는다. 다르게 나오면 실패로 기록하고 원인을 적는다.

---

## 10. 작성자가 아닌 팀원의 재현

발제: *"작성자가 아닌 팀원이 README만 보고 실행합니다. 확인자·날짜·기준 커밋·결과를 남기고 누락된 경로·설정을 수정합니다."*

1. 기록하지 않은 팀원이 **README 18~20절과 recordings/README만** 보고 진행한다. 이 가이드나 구두 설명에 의존하면 README 재현 확인이 아니다. 막힌 곳은 메모한다.
2. 다운로드 후 무결성 확인:
   ```bash
   R=$PWD/recordings/$RUN                                              # lv2_module5 폴더에서
   (cd ~/Downloads && sha256sum -c $R/ARCHIVE.sha256)                  # 압축본
   mkdir -p ~/bags && tar -xzf ~/Downloads/$RUN.tar.gz -C ~/bags \
     && (cd ~/bags/$RUN && sha256sum -c $R/SHA256SUMS)                 # metadata.yaml·데이터 파일
   ```
3. 6절 안전 → `tools/bag_replay.sh reanalysis ~/bags/$RUN` (8절 B) → `tools/bag_replay.sh reprocess ~/bags/$RUN` (7절 A) → 9절 비교 결과(`compare.txt`) 확인.
   B를 먼저 하면 A의 대조 기준(`<RUN>_reanalysis.csv`)이 생긴다.
4. `recordings/README.md` 4절 표에 1행 기록: run ID, 재현 종류(A/B), 명령(`results/logs/replay/.../run.txt`), **확인자, 날짜, 기준 commit**(`run.txt`의 `git_commit`), sha256 확인 결과, 결과(행 수·대조 PASS/FAIL·빠진 프레임).
5. README에서 빠졌거나 틀린 경로·옵션은 Issue → PR로 고친다(예: `--topics` 형식, source 순서). 이 PR이 재현 팀원의 "본인 PR" 또는 리뷰 증거가 될 수 있다.

---

## 11. report.md · team.md 작성

report.md **문제 5**는 7단계 틀로 쓴다. 각 칸에 넣을 내용:

| 단계 | 넣을 내용 |
|---|---|
| 1 구현 내용 | 성취도 45. 기록 토픽, A·B 재현 구성, 모터 OFF 3겹, 사용 도구 위치 |
| 2 실행 조건 | 장비, 기록 위치(SD/SSD), 기간, 시나리오(4.3절), 재생 rate, 도메인 |
| 3 결과물 | bag 링크·sha256, `recordings/<RUN>/`(info·check·params), 4개 CSV, 재처리 이미지 폴더, `run.txt`의 실제 record/play 명령 |
| 4 측정 결과 | `compare.txt` 대조 결과(공통 프레임·빠진 프레임·최대 차), `analyze.txt` 실시간 vs 재분석 표 |
| 5 해석 | **A와 B의 차이**: A는 검출기 재현성(입력 → 출력), B는 지표 계산 재현성(출력 → 지표). 9.3절의 정상 차이와 그 밖의 차이 원인 |
| 6 심화 | 도전 E 수행 여부 (14절) |
| 7 한계 | 오프라인 재현은 실제 하드웨어 폐루프 시연이 아님. 기록 부하로 FPS가 변했는지, 프레임 손실, 시리얼 로그와 bag의 시계가 달라 직접 맞추지 않은 점 |

team.md: 기록·업로드·재현·리뷰 담당자와 각 PR 링크, 재현 확인자를 적는다.

---

## 12. 자주 나는 문제

| 증상 | 원인 | 해결 |
|---|---|---|
| `ros2 bag info`에 `/control/pan_tilt_cmd`가 없음, 재생 시 타입 오류 | 워크스페이스 미source | 3.2절 source 후 다시 |
| `metadata.yaml`이 없음 | SSH 끊김·강제 종료 | `ros2 bag reindex ~/bags/$RUN` 시도. 안 되면 다시 기록 (3.4절 tmux) |
| image_raw 개수가 30×D보다 크게 적음 | 저장 속도 부족 | SSD에 기록, 다른 무거운 프로세스 종료 |
| 재처리 CSV가 0행 | perception이 영상을 못 받음: 도메인 다름, `camera_config` 누락, 재생을 먼저 시작 | 같은 `ROS_DOMAIN_ID`, `-p camera_config:=...`, `--delay 2` |
| 재처리 CSV 행 수가 영상보다 적음 | 검출기가 재생 속도를 못 따라감 | `--rate 0.5` 또는 더 낮게 |
| `tracking_logger`가 시작 거부 (`FileExistsError`) | 같은 run-id CSV가 이미 있음 (덮어쓰기 방지) | 새 run-id 사용 (`success_01_reanalysis_2`) |
| `receive_time_s`가 현재 시각 | `use_sim_time:=true` 누락 | 노드에 `--ros-args -p use_sim_time:=true`, play에 `--clock` |
| `opencr_node`가 시작 거부 (CSV 존재) | `csv_path` 파일이 이미 있음 | 새 RUN 이름 |
| 실시간 CSV와 재분석 CSV의 처음 몇 행이 다름 | 기록기·bag 시작 시점 차이 | 공통 `stamp_ns` 구간만 비교 (`bag_tool compare`가 자동으로 함) |
| `bag_record.sh`: "필수 토픽이 발행되지 않음" | 추적·카메라가 안 돌거나 다른 `ROS_DOMAIN_ID` | 14절 실행 확인, 같은 도메인. 일부러 빼고 기록할 때만 `--force` |
| `bag_record.sh`: "/opencr_node 파라미터를 읽지 못함" | opencr_node 미실행 (예: 인지만 기록) | 경고일 뿐 기록은 계속. 이유를 source.txt 옆에 메모 |
| `bag_replay.sh`: "OpenCR USB가 연결돼 있음" | 물리 차단 안 됨 | USB 분리 또는 모터 전원 OFF 후, 전원 OFF만 했다면 `--motor-off-confirmed` |
| `bag_replay.sh`: "도메인 99에 이미 노드가 있음" | 이전 재생 노드가 남았거나 다른 사람이 같은 도메인 사용 | 남은 프로세스 종료 또는 `--domain 98` |
| `bag_replay.sh`: "이미 있음 ... _reprocess.csv" | 같은 run을 다시 재현 | `--suffix _2` (기존 결과는 지우지 않음) |
| `compare`: state 불일치 10 % 넘음 경고 | B에서 `/tracking_status`가 기록·재생되지 않음 | `check.txt`의 토픽 개수 확인 |

---

## 13. 현재 도구 기준 주의점

| 항목 | 현재 동작 | 운영 방법 |
|---|---|---|
| `tracking_logger` (A) | 상태·명령 열은 `/tracking_status`·`/control/pan_tilt_cmd`에서 채운다. A 재생에는 이 토픽이 없어 빈 값 | A는 검출·ex·ey만 비교 (9.3절). 원본 상태를 섞어 재생하지 않는다 |
| `eval_frames.py` (A 이미지) | `--output-dir`를 주면 그 폴더에만 저장하고 `eval_runs.csv`에 쓰지 않음 (`bag_replay.sh --images`가 사용) | 재처리 이미지는 반드시 `--output-dir`(또는 스크립트)로. 사람 대조 지표(`eval_score.py`)에 넣지 않음 |
| `tracker.yaml` `use_depth: true` | 깊이 영상을 구독 | A에서는 `-p use_depth:=false` (스크립트가 자동 지정) |
| 기록 당시 파라미터 | `bag_record.sh`가 `ros2 param dump`로 저장. 노드가 없으면 저장 못 함 | 기록 전에 추적 노드가 모두 떠 있는지 확인. 없으면 재처리는 현재 tracker.yaml로 하고 report에 적음 |
| 시리얼 CSV `monotonic_sec` | Pi 단조 시계 (bag의 ROS 시각과 다름) | **run ID로만 연결**하고 두 시계를 빼서 지연을 계산하지 않음 (발제: 과거 bag 시각과 벽시계 혼용 금지). 사건 순서(FAULT, STOP)로만 대응 |
| `ros2 bag record/play` 옵션 | `--topics`, `-s`, `--delay`, `--rate`, `--clock`은 Jazzy 이후 기준 | 첫 사용 때 Pi에서 `--help`로 확인. 스크립트는 `--topics` 미지원이면 위치 인자로 바꿈 |
| bridge의 `EVENT LIMIT` 처리 | 펌웨어는 경계에서 ARM 유지 정지(`EVENT LIMIT STOPPED`), bridge는 FAULT로 처리 | 기록 시나리오는 경계 안쪽에서 (4.3절). 동작을 맞출지는 제어·통합 담당이 결정 |

---

## 14. (선택) 도전 E — 고정 bag 회귀 비교

필수(1~13절)를 마친 뒤에만 한다.

1. `success_01`·`lost_01`을 **기준 bag**으로 고정한다. sha256, 기준 commit, 설정을 recordings 표에 적는다.
2. 검출기·인터페이스·제어 설정 중 **하나만** 바꾼다(예: HSV S 하한). 바꾸기 전에 Issue에 변경 변수·비교 지표·완료 기준을 적는다.
3. 변경 전·후 commit에서 각각 7절 A를 실행한다. 바꾼 설정이 적용되도록 기록 당시 파라미터 대신 현재 설정을 쓴다:
   `tools/bag_replay.sh reprocess ~/bags/$RUN --current-config --suffix _<commit>`
4. `bag_tool.py compare`(9.2절)와 `analyze_tracking`으로 검출 일치·노드 검출 비율·ex/ey 차를 비교한다. 변경 후에는 FAIL이 나는 것이 정상일 수 있다 — 차이가 개선인지 회귀인지 해석한다.
5. 제출: 기준·변경 commit, 재생 명령, 전후 표·그래프, 회귀 여부와 원인. 개선되지 않았어도 기록한다.

> 제어 설정(Kp 등)의 회귀는 bag 재생만으로 판정할 수 없다. 기록된 영상은 당시 카메라 움직임의 결과라서 폐루프가 다시 돌지 않는다. 이 한계를 보고서에 명시한다.

