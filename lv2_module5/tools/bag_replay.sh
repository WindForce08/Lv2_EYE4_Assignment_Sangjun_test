#!/usr/bin/env bash
# ROS 2 bag 재현 (문제 5, 담당: 기록하지 않은 팀원) — 실제 모터 출력 없이 실행
#
# 사용: tools/bag_replay.sh <reprocess|reanalysis> <BAG_DIR> [옵션]
#   reprocess   A. 입력 재처리: bag 영상만 → perception_node(-r /target:=/target_replay) → <RUN>_reprocess.csv
#   reanalysis  B. 결과 재분석: bag의 /target·상태·명령 → tracking_logger → <RUN>_reanalysis.csv
# 옵션
#   --run-id RUN           실행 ID (기본: bag 폴더 이름)
#   --rate R               재생 속도 (기본 reprocess 0.5 — 검출기가 프레임을 놓치지 않게, reanalysis 1.0)
#   --domain N             격리 ROS_DOMAIN_ID (기본 99, 실시간 추적에 쓴 도메인과 달라야 함)
#   --params FILE          A의 perception 파라미터 파일 (기본: recordings/<RUN>/params/perception_node.yaml
#                          = 기록 당시 실제 값, 없으면 tracker.yaml)
#   --current-config       기록 당시 값 대신 현재 tracker.yaml 사용 (도전 E: 설정 변경 전후 회귀 비교)
#   --suffix S             결과 이름 <RUN>_<mode><S> (같은 run을 다시 재현할 때, 예: _2)
#   --images N             (reprocess) 재처리 이미지 N장을 재생 구간에 고르게 저장 → results/images/replay/<RUN>_reprocess/
#   --motor-off-confirmed  OpenCR USB가 연결돼 있어도 진행 (모터 전원 OFF를 직접 확인했을 때만)
#
# 안전 (가이드 6절): opencr_node·control_node 실행 중이면 거부, OpenCR USB 연결 시 거부(확인 옵션 제외),
#   격리 도메인(ROS_DOMAIN_ID, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)에 다른 노드가 있으면 거부.
#   A는 명령 토픽을 재생하지 않는다. B는 재생하지만 받는 쪽은 tracking_logger뿐이다.
# 결과
#   results/logs/verification/<RUN>_<mode>.csv          tracking_logger CSV (Git에 올림)
#   results/logs/replay/<RUN>_<mode>_<시각>/            실행 명령·환경·노드 로그·analyze·compare 결과 (Git에 올림)
#   results/images/replay/<RUN>_reprocess/              --images: 원본·검출 그림 + 프레임 목록 CSV (Git에 올림)
set -euo pipefail

LV2="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL="$LV2/tools/bag_tool.py"
PKG_CFG="$LV2/ros2_ws/src/realsense_tracker/config"
VERIFY="$LV2/results/logs/verification"

die() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "WARN: $*" >&2; }
usage() { sed -n '2,24p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

[[ $# -ge 2 ]] || usage
MODE="$1"; BAG="$2"; shift 2
[[ "$MODE" == reprocess || "$MODE" == reanalysis ]] || usage
RUN=""; RATE=""; DOMAIN=99; PARAMS=""; CURRENT=0; SUFFIX=""; MOTOR_OFF=0; IMAGES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) RUN="${2:?}"; shift 2 ;;
    --rate) RATE="${2:?}"; shift 2 ;;
    --domain) DOMAIN="${2:?}"; shift 2 ;;
    --params) PARAMS="${2:?}"; shift 2 ;;
    --current-config) CURRENT=1; shift ;;
    --suffix) SUFFIX="${2:?}"; shift 2 ;;
    --images) IMAGES="${2:?}"; shift 2 ;;
    --motor-off-confirmed) MOTOR_OFF=1; shift ;;
    -h|--help) usage ;;
    *) die "알 수 없는 옵션: $1" ;;
  esac
done
BAG="$(cd "$BAG" 2>/dev/null && pwd)" || die "bag 폴더 없음"
RUN="${RUN:-$(basename "$BAG")}"
[[ "$RUN" =~ ^[A-Za-z0-9_-]+$ ]] || die "RUN은 영문·숫자·_- 만: $RUN"
RATE="${RATE:-$([[ $MODE == reprocess ]] && echo 0.5 || echo 1.0)}"
OUT_ID="${RUN}_${MODE}${SUFFIX}"
CSV="$VERIFY/$OUT_ID.csv"
[[ "$IMAGES" =~ ^[0-9]+$ ]] || die "--images는 정수"
[[ $IMAGES -eq 0 || $MODE == reprocess ]] || die "--images는 reprocess에서만"
IMG_DIR="$LV2/results/images/replay/$OUT_ID"
[[ $IMAGES -eq 0 || ! -e "$IMG_DIR" ]] || die "이미 있음: $IMG_DIR — --suffix 사용"

# ── 사전 확인 ────────────────────────────────────────────────────────────────
command -v ros2 >/dev/null || die "ros2 없음 — source /opt/ros/<distro>/setup.bash"
python3 -c "import realsense_tracker_interfaces.msg" 2>/dev/null \
  || die "PanTiltCommand 타입 없음 — source <lv2_module5>/ros2_ws/install/setup.bash (README 8절)"
PREFIX="$(ros2 pkg prefix realsense_tracker 2>/dev/null)" || die "realsense_tracker 패키지 없음 — 워크스페이스 빌드·source"
[[ ! -e "$CSV" ]] || die "이미 있음: $CSV — 다시 재현하려면 --suffix _2"
python3 "$TOOL" check "$BAG" || die "bag 확인 실패 (파일 누락 등) — 다운로드·압축 해제·sha256sum -c 확인"

# ── 안전: 모터 출력 끄기 (3겹) ────────────────────────────────────────────────
if pgrep -af -- '(^|[ /])(opencr_node|control_node)( |$)' >&2; then
  die "opencr_node/control_node 실행 중 — 종료 후 다시 (pkill -f opencr_node; pkill -f control_node)"
fi
if compgen -G "/dev/serial/by-id/*OpenCR*" >/dev/null; then
  if [[ $MOTOR_OFF -eq 1 ]]; then warn "OpenCR USB 연결됨 — --motor-off-confirmed로 진행 (모터 전원 OFF를 확인했다고 기록)"
  else die "OpenCR USB가 연결돼 있음 — 카메라를 지지한 뒤 USB를 뽑거나 모터 전원을 끄고, 끈 경우에만 --motor-off-confirmed"; fi
fi
export ROS_DOMAIN_ID="$DOMAIN" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
if ! OTHERS="$(timeout 20 ros2 node list --no-daemon --spin-time 2 2>/dev/null)"; then   # 옵션 미지원 배포판 대비
  OTHERS="$(timeout 20 ros2 node list 2>/dev/null || true)"
fi
[[ -z "$OTHERS" ]] || die "도메인 $DOMAIN에 이미 노드가 있음: $(tr '\n' ' ' <<<"$OTHERS") — 다른 --domain 사용"

# ── 재생 구성 ────────────────────────────────────────────────────────────────
read -r -a TOPICS <<<"$(python3 "$TOOL" topics "$MODE")"
[[ ${#TOPICS[@]} -gt 0 ]] || die "재생 토픽을 읽지 못함"
if [[ $MODE == reprocess ]]; then
  SNAP="$LV2/recordings/$RUN/params/perception_node.yaml"
  if [[ -z "$PARAMS" ]]; then
    if [[ $CURRENT -eq 0 && -f "$SNAP" ]]; then PARAMS="$SNAP"
    else
      PARAMS="$PKG_CFG/tracker.yaml"
      [[ $CURRENT -eq 1 ]] || warn "기록 당시 파라미터($SNAP)가 없어 현재 tracker.yaml 사용 — 기록 당시와 다르면 대조가 달라질 수 있음"
    fi
  fi
  [[ -f "$PARAMS" ]] || die "파라미터 파일 없음: $PARAMS"
  TARGET_TOPIC=/target_replay
else
  TARGET_TOPIC=/target
fi

LOGDIR="$LV2/results/logs/replay/${OUT_ID}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
PERCEPTION_CMD=("$PREFIX/lib/realsense_tracker/perception_node" --ros-args --params-file "${PARAMS:-}"
  -p camera_config:="$PKG_CFG/camera.yaml" -p use_depth:=false -p use_sim_time:=true
  -p publish_debug_image:=true -r /target:=/target_replay)
LOGGER_CMD=(python3 "$LV2/tools/tracking_logger.py" --run-id "$OUT_ID" --target-topic "$TARGET_TOPIC"
  --ros-args -p use_sim_time:=true)
PLAY_CMD=(ros2 bag play "$BAG" --clock --delay 2 --rate "$RATE" --topics "${TOPICS[@]}")
FRAMES_CMD=()
if [[ $IMAGES -gt 0 ]]; then
  # 재생 실제 시간(bag 기간 ÷ rate)의 80% 동안 고르게 N장. eval_frames는 실제 경과 시간으로 간격을 나눈다.
  SPAN=$(python3 -c "import yaml,sys;i=yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information'];print(max(1.0,round(i['duration']['nanoseconds']/1e9/float(sys.argv[2])*0.8,1)))" "$BAG/metadata.yaml" "$RATE")
  FRAMES_CMD=(python3 "$LV2/tools/eval_frames.py" --scene visible --target-topic /target_replay --no-view
    --frames "$IMAGES" --duration "$SPAN" --output-dir "$IMG_DIR" --note "replay $RUN reprocess (not human-eval set)")
fi
{
  echo "mode: $MODE"
  echo "run_id: $RUN"
  echo "output_csv: results/logs/verification/$OUT_ID.csv"
  echo "replayed_at: $(date -Iseconds)"
  echo "host: $(hostname)"
  echo "git_commit: $(git -C "$LV2" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty_files: |"
  git -C "$LV2" status --short 2>/dev/null | sed 's/^/  /' || true
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "ros_domain_id: $ROS_DOMAIN_ID (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)"
  echo "motor_output: opencr_node/control_node not running; OpenCR USB $(compgen -G "/dev/serial/by-id/*OpenCR*" >/dev/null && echo "connected, motor power OFF confirmed by operator" || echo "not connected")"
  echo "bag_sha256_check: (다운로드했다면 recordings/$RUN/SHA256SUMS로 sha256sum -c 결과를 recordings/README 4절에 기록)"
  [[ $MODE == reprocess ]] && echo "perception_params: ${PARAMS#"$LV2"/}" && echo "perception_command: ${PERCEPTION_CMD[*]}"
  [[ $IMAGES -gt 0 ]] && echo "frames_command: ${FRAMES_CMD[*]}"
  echo "logger_command: ${LOGGER_CMD[*]}"
  echo "play_command: ${PLAY_CMD[*]}"
} >"$LOGDIR/run.txt"

# ── 실행 ─────────────────────────────────────────────────────────────────────
# job control: 백그라운드 노드가 SIGINT를 무시하지 않게 하고(정상 종료 → CSV 저장), Ctrl+C는 재생에만 전달한다.
set -m
PIDS=()
stop() {  # SIGINT → 10초 기다림 → SIGTERM
  local pid=$1
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 100); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
  warn "PID $pid가 SIGINT에 끝나지 않아 SIGTERM"; kill -TERM "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
}
cleanup() { for pid in "${PIDS[@]}"; do stop "$pid"; done; }
trap cleanup EXIT

if [[ $MODE == reprocess ]]; then
  "${PERCEPTION_CMD[@]}" >"$LOGDIR/perception.log" 2>&1 & PIDS+=($!)
fi
if [[ $IMAGES -gt 0 ]]; then
  "${FRAMES_CMD[@]}" >"$LOGDIR/frames.log" 2>&1 & PIDS+=($!); FRAMES_PID=$!
fi
"${LOGGER_CMD[@]}" >"$LOGDIR/logger.log" 2>&1 & PIDS+=($!)
sleep 3
for pid in "${PIDS[@]}"; do
  kill -0 "$pid" 2>/dev/null || { tail -20 "$LOGDIR"/*.log >&2; die "노드가 시작 직후 종료됨 — 위 로그 확인"; }
done

echo "재생: ${PLAY_CMD[*]}"
set +e
"${PLAY_CMD[@]}" 2>&1 | tee "$LOGDIR/play.log"
PLAY=${PIPESTATUS[0]}
set -e
sleep 2                                    # 마지막 메시지 처리 여유
if [[ $IMAGES -gt 0 ]]; then               # eval_frames는 N장을 다 저장하면 스스로 끝남. 남아 있으면 저장한 장까지만
  for _ in $(seq 50); do kill -0 "$FRAMES_PID" 2>/dev/null || break; sleep 0.1; done
fi
for ((i=${#PIDS[@]}-1; i>=0; i--)); do stop "${PIDS[$i]}"; done   # logger 먼저 (CSV 저장), 그 다음 perception
PIDS=()
set +m
[[ $PLAY -eq 0 ]] || warn "ros2 bag play exit $PLAY (play.log 확인)"
[[ -f "$CSV" ]] || die "CSV가 만들어지지 않음 — logger.log 확인"
ROWS=$(( $(wc -l <"$CSV") - 1 ))
IMG_LINE=""
[[ $IMAGES -gt 0 ]] && IMG_LINE=$'\n'"  이미지   ${IMG_DIR#"$LV2"/}/ ($(ls "$IMG_DIR"/*_det.png 2>/dev/null | wc -l)장, frames.log)"

# ── 결과 ─────────────────────────────────────────────────────────────────────
python3 "$LV2/tools/analyze_tracking.py" "$CSV" | tee "$LOGDIR/analyze.txt"
if [[ $MODE == reprocess ]]; then
  BASE="$VERIFY/${RUN}_reanalysis.csv"; [[ -f "$BASE" ]] || BASE="$VERIFY/$RUN.csv"
  COMPARE_ARGS=()
else
  BASE="$VERIFY/$RUN.csv"
  COMPARE_ARGS=(--state)
fi
COMPARE=skipped
if [[ -f "$BASE" ]]; then
  echo; echo "대조: ${BASE#"$LV2"/} ↔ ${CSV#"$LV2"/}"
  set +e
  python3 "$TOOL" compare "$BASE" "$CSV" "${COMPARE_ARGS[@]}" | tee "$LOGDIR/compare.txt"
  COMPARE=$([[ ${PIPESTATUS[0]} -eq 0 ]] && echo PASS || echo FAIL)
  set -e
else
  warn "대조 기준 CSV 없음 (${BASE#"$LV2"/}) — 기록 때 tracking_logger를 같은 RUN으로 실행했는지 확인"
fi

cat <<EOF

완료: $MODE $RUN — $ROWS행, 대조 $COMPARE
  CSV      ${CSV#"$LV2"/}${IMG_LINE}
  로그     ${LOGDIR#"$LV2"/}/ (run.txt, *.log, analyze.txt, compare.txt)
다음 할 일
  - recordings/README.md 4절 표에 1행: run ID, 재현 종류($([[ $MODE == reprocess ]] && echo A || echo B)), 명령(run.txt), 확인자, 날짜, 기준 commit, 결과($COMPARE, 행 수·빠진 프레임)
EOF
if [[ $MODE == reprocess && $IMAGES -eq 0 ]]; then
  echo "  - 재처리 이미지(발제 결과물)가 필요하면: tools/bag_replay.sh reprocess $BAG --images 10 --suffix _img"
fi
[[ $COMPARE != FAIL ]] || exit 1   # 대조 불일치는 exit 1 (결과 파일은 남김)
