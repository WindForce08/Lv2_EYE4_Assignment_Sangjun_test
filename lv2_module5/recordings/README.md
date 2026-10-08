# recordings — 문제 5 bag 기록·재현

bag 파일은 용량 때문에 Git에 올리지 않는다(.gitignore). 팀 공유 드라이브에 올리고, 이 폴더에는 **증거(메타데이터·체크섬·실제 파라미터)** 와 링크만 둔다.
실제 bag을 만들기 전에는 아래 표를 채우지 않는다 (현재 기록된 bag 없음).
자세한 이유·시나리오·비교 해석: [../docs/ROS 2 bag 구현 가이드 문제 5.md](<../docs/ROS 2 bag 구현 가이드 문제 5.md>)

| 도구 | 하는 일 |
|---|---|
| [`../tools/bag_record.sh`](../tools/bag_record.sh) | 사전 확인 → 실제 파라미터 저장 → `ros2 bag record` → info·메시지 수 확인·sha256 → `recordings/<RUN>/` |
| [`../tools/bag_replay.sh`](../tools/bag_replay.sh) | 모터 OFF 확인 → A 입력 재처리 / B 결과 재분석 → CSV·로그 → 자동 대조 |
| [`../tools/bag_tool.py`](../tools/bag_tool.py) | 토픽 목록(`topics`), bag 확인(`check`), CSV 프레임 대조(`compare`) — ROS 없이 실행 |
| [`../config/bag.yaml`](../config/bag.yaml) | 기록·재생 토픽, 기대 주기, 기간 기준 (카메라 토픽 이름은 camera.yaml) |

## 1. 기록 (Raspberry Pi, 실제 추적 중)

사전조건: [README 14절](../README.md#14-실제-tracking-실행)로 추적이 돌고 있음. opencr_node `csv_path:=$HOME/runs/<RUN>_serial.csv`, `tools/tracking_logger.py --run-id <RUN>` 실행 중.
SSH가 끊겨도 기록이 정상 종료되도록 `tmux` 안에서 실행한다.

```bash
cd ~/git/Lv2_EYE4_Assignment/lv2_module5        # ROS + ros2_ws/install/setup.bash source 후
tools/bag_record.sh success_01 --duration 20 --archive   # 대표 성공 장면 (Ctrl+C로 일찍 끝낼 수 있음)
tools/bag_record.sh lost_01    --duration 20 --archive   # 소실·복귀 장면 (2 s 가림 → 시야 안 재등장)
```

| 결과 | 위치 | Git |
|---|---|---|
| bag (`metadata.yaml` + `*.mcap`) | `~/bags/<RUN>/` (기본, `--out-dir`·`$BAG_DIR`로 변경. 저장소 안은 거부) | ✗ |
| 업로드용 압축본 (`--archive`) | `~/bags/<RUN>.tar.gz` | ✗ |
| `source.txt` | 기록 시각·호스트·**기준 commit·커밋 안 된 변경**·ROS 배포판·도메인·기록 명령·bag 크기 | ✓ |
| `info.txt`, `check.txt` | `ros2 bag info`, 토픽별 메시지 수·주기·기간 판정 (`bag_tool check`) | ✓ |
| `SHA256SUMS`, `ARCHIVE.sha256` | bag 파일별 sha256, 압축본 sha256 | ✓ |
| `params/<node>.yaml` | 기록 시작 때 `ros2 param dump` — `-p`로 바꾼 값까지 포함한 **실제 사용 설정** | ✓ |
| `config/*.yaml` | 그 시점의 설정 파일 사본 (tracker·camera·control·opencr_live·bag) | ✓ |

기록 후 할 일: ① `results/logs/verification/<RUN>.csv`(tracking_logger)와 `<RUN>_serial.csv`(시리얼) 커밋 ② 압축본을 공유 드라이브에 올리고 **평가자 계정으로 열리는지** 다른 팀원이 확인 ③ 아래 3절 표에 1행.
`check.txt`가 FAIL(파일 누락·필수 토픽 없음)이거나 기간이 10~30 s 밖이면 번호를 올려 다시 기록한다. 실패한 기록도 지우지 않는다.

## 2. 재현 (모터 출력 OFF, 작성자가 아닌 팀원)

```bash
# 다운로드했다면 무결성 확인 (R = 이 저장소의 recordings/<RUN>)
R=$PWD/recordings/success_01
(cd ~/Downloads && sha256sum -c $R/ARCHIVE.sha256)                                   # 압축본
mkdir -p ~/bags && tar -xzf ~/Downloads/success_01.tar.gz -C ~/bags \
  && (cd ~/bags/success_01 && sha256sum -c $R/SHA256SUMS)                            # metadata.yaml·데이터 파일

# 안전: 카메라 지지 → OpenCR USB 분리 또는 모터 전원 OFF, opencr_node·control_node 종료 (스크립트가 다시 확인)
tools/bag_replay.sh reanalysis ~/bags/success_01            # B 결과 재분석 → <RUN>_reanalysis.csv, 실시간 CSV와 자동 대조
tools/bag_replay.sh reprocess  ~/bags/success_01            # A 입력 재처리 → <RUN>_reprocess.csv, B 결과와 자동 대조
tools/bag_replay.sh reprocess  ~/bags/success_01 --images 10 --suffix _img   # 재처리 이미지 10장
```

| 재현 | 재생 토픽 | 검출기 | 출력 | 대조 기준 | 기대 |
|---|---|---|---|---|---|
| A. 입력 재처리 | 컬러 영상·camera_info만 | 실행 (`-r /target:=/target_replay`, 기록 당시 `params/perception_node.yaml`, `use_sim_time`, `--clock`) | `<RUN>_reprocess.csv` | `<RUN>_reanalysis.csv` | 공통 프레임 검출·ex·ey 완전히 같음 |
| B. 결과 재분석 | `/target`, `/tracking_status`, `/control/pan_tilt_cmd` | 실행 안 함 | `<RUN>_reanalysis.csv` | `<RUN>.csv` (실시간) | 공통 프레임 값 같음, 지표 거의 같음 |

- 저장된 `/target`과 새 검출 결과를 같은 토픽에 섞지 않는다. 저장된 `/target`을 보기만 한 것은 입력 재처리가 아니다.
- 재생 로그·명령·대조 결과: `results/logs/replay/<RUN>_<mode>_<시각>/` (`run.txt`, `*.log`, `analyze.txt`, `compare.txt`). 재처리 이미지: `results/images/replay/<RUN>_reprocess<suffix>/`.
- 재처리 CSV에는 상태·명령이 없어 유효 추적 비율 0 %, RMSE `nan`이 정상이다 — A는 검출·ex·ey만 비교한다.
- 수동 명령(스크립트 없이)과 각 옵션의 이유: [README 19절](../README.md#19-bag-replay), 가이드 7~8절.

## 3. 메타데이터 (bag마다 1행 — 실제 값만)

값은 `recordings/<RUN>/source.txt`, `info.txt`, `check.txt`, `SHA256SUMS`에서 옮긴다.

| run ID | 목적 | bag 위치(링크) | 기간 [s] | 토픽·메시지 수 | 해상도 | 설정 | 기준 commit (dirty?) | 크기 | 압축본 sha256 | check | 시리얼 로그 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| (없음) | | | | | | `recordings/<RUN>/params/` | | | | | |

## 4. 재현 기록 (작성자가 아닌 팀원)

| run ID | 재현 종류(A/B) | 명령 (`run.txt`) | 확인자 | 날짜 | 기준 commit | sha256 확인 | 결과 (행 수, 대조 PASS/FAIL, 빠진 프레임·원인) |
|---|---|---|---|---|---|---|---|
| (없음) | | | | | | | |

README만 보고 막힌 곳이 있으면 Issue → PR로 README를 고치고 그 링크를 결과 칸에 적는다.
