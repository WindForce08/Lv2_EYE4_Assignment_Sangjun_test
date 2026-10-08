#!/usr/bin/env bash
# ROS 2 bag 기록 (문제 5, 담당: 통합) — Raspberry Pi에서 실제 추적이 돌고 있을 때 실행
#
# 사용: tools/bag_record.sh <RUN> [--duration SEC] [--out-dir DIR] [--archive] [--force]
#   RUN         실행 ID. 성공 장면 success_NN, 소실·복귀 장면 lost_NN (영문·숫자·_- 만)
#   --duration  SEC초 뒤 자동 종료 (없으면 Ctrl+C로 종료)
#   --out-dir   bag 상위 폴더 (기본 $BAG_DIR 또는 ~/bags). Git 저장소 안은 거부
#   --archive   기록 후 <out-dir>/<RUN>.tar.gz 생성 (업로드용) + 체크섬
#   --force     사전 확인(필수 토픽 없음·저장 공간 부족) 실패를 무시하고 진행
#
# 하는 일
#   1. 사전 확인: ROS·워크스페이스 source, 같은 RUN 없음, 저장 공간, 필수 토픽이 지금 발행 중인지
#   2. 실행 중인 노드의 실제 파라미터 저장 (ros2 param dump) — 실행 시 -p 로 바꾼 값까지 증거로 남김
#   3. ros2 bag record (토픽: config/bag.yaml + camera.yaml)
#   4. 기록 후: ros2 bag info, bag_tool check(메시지 수·기간), 파일별 sha256, 기준 commit
# 결과
#   <out-dir>/<RUN>/                bag (metadata.yaml + 데이터 파일) — Git에 올리지 않음, 외부 저장소로
#   recordings/<RUN>/               Git에 올리는 증거: source.txt, info.txt, check.txt, SHA256SUMS,
#                                   (ARCHIVE.sha256), params/*.yaml, config/*.yaml
set -euo pipefail

LV2="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL="$LV2/tools/bag_tool.py"
PKG_CFG="$LV2/ros2_ws/src/realsense_tracker/config"

die() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "WARN: $*" >&2; }
usage() { sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

[[ $# -ge 1 ]] || usage
RUN="$1"; shift
[[ "$RUN" =~ ^[A-Za-z0-9_-]+$ ]] || die "RUN은 영문·숫자·_- 만: $RUN"
DURATION=""; OUT_DIR="${BAG_DIR:-$HOME/bags}"; ARCHIVE=0; FORCE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="${2:?}"; shift 2 ;;
    --out-dir) OUT_DIR="${2:?}"; shift 2 ;;
    --archive) ARCHIVE=1; shift ;;
    --force) FORCE=1; shift ;;
    -h|--help) usage ;;
    *) die "알 수 없는 옵션: $1" ;;
  esac
done
[[ -z "$DURATION" || "$DURATION" =~ ^[0-9]+$ ]] || die "--duration은 정수 초"
check_or_force() { if [[ $FORCE -eq 1 ]]; then warn "$* (--force로 계속)"; else die "$* (무시하려면 --force)"; fi; }

# ── 1. 사전 확인 ─────────────────────────────────────────────────────────────
command -v ros2 >/dev/null || die "ros2 없음 — source /opt/ros/<distro>/setup.bash"
python3 -c "import realsense_tracker_interfaces.msg" 2>/dev/null \
  || die "PanTiltCommand 타입 없음 — source <lv2_module5>/ros2_ws/install/setup.bash (README 8절)"
PROBE="$OUT_DIR"; while [[ ! -d "$PROBE" ]]; do PROBE="$(dirname "$PROBE")"; done   # 폴더를 만들기 전에 검사
if REPO_TOP="$(git -C "$PROBE" rev-parse --show-toplevel 2>/dev/null)"; then
  die "bag 폴더가 Git 저장소($REPO_TOP) 안에 있음 — metadata.yaml이 커밋될 수 있으므로 저장소 밖(~/bags, USB SSD)을 쓴다"
fi
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"
BAG="$OUT_DIR/$RUN"
META="$LV2/recordings/$RUN"
[[ ! -e "$BAG" ]] || die "이미 있음: $BAG — 다시 찍으면 번호를 올린다 (기존 기록은 지우지 않음)"
[[ ! -e "$META" ]] || die "이미 있음: $META"

STORAGE="${BAG_STORAGE:-$(python3 -c "import yaml;print(yaml.safe_load(open('$LV2/config/bag.yaml'))['storage'])")}"
BPS="$(python3 -c "import yaml;print(yaml.safe_load(open('$LV2/config/bag.yaml'))['bytes_per_sec_estimate'])")"
NEED=$(( BPS * ${DURATION:-30} * 2 ))
AVAIL=$(df --output=avail -B1 "$OUT_DIR" | tail -1 | tr -d ' ')
echo "저장 공간: 여유 $((AVAIL / 1000000)) MB, 필요(예상×2) $((NEED / 1000000)) MB — $OUT_DIR"
[[ $AVAIL -ge $NEED ]] || check_or_force "저장 공간 부족"

read -r -a TOPICS <<<"$(python3 "$TOOL" topics record)"
read -r -a REQUIRED <<<"$(python3 "$TOOL" topics record --required)"
[[ ${#TOPICS[@]} -gt 0 ]] || die "기록 토픽을 읽지 못함 — python3 $TOOL topics record 오류 확인"
LIVE="$(timeout 15 ros2 topic list 2>/dev/null || true)"
for t in "${TOPICS[@]}"; do
  if ! grep -qxF -- "$t" <<<"$LIVE"; then
    if printf '%s\n' "${REQUIRED[@]}" | grep -qxF -- "$t"; then check_or_force "필수 토픽이 발행되지 않음: $t"
    else warn "선택 토픽 없음(기록은 계속): $t"; fi
  fi
done

if ros2 bag record --help 2>/dev/null | grep -q -- '--topics'; then TOPIC_ARGS=(--topics "${TOPICS[@]}")
else TOPIC_ARGS=("${TOPICS[@]}"); warn "이 rosbag2는 --topics를 지원하지 않아 위치 인자로 넘김"; fi
RECORD_CMD=(ros2 bag record -o "$BAG" -s "$STORAGE" "${TOPIC_ARGS[@]}")

# ── 2. 실행 환경·실제 파라미터 기록 ───────────────────────────────────────────
mkdir -p "$META/params" "$META/config"
for node in perception_node control_node opencr_node; do
  if timeout 15 ros2 param dump "/$node" >"$META/params/$node.yaml" 2>/dev/null && [[ -s "$META/params/$node.yaml" ]]; then
    echo "파라미터 저장: params/$node.yaml"
  else
    rm -f "$META/params/$node.yaml"; warn "/$node 파라미터를 읽지 못함 (노드가 없거나 다른 도메인)"
  fi
done
cp "$PKG_CFG"/{tracker,camera,control,opencr_live}.yaml "$LV2/config/bag.yaml" "$META/config/"
{
  echo "run_id: $RUN"
  echo "recorded_at: $(date -Iseconds)"
  echo "host: $(hostname)"
  echo "bag_path: $BAG"
  echo "git_commit: $(git -C "$LV2" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_branch: $(git -C "$LV2" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty_files: |"
  git -C "$LV2" status --short 2>/dev/null | sed 's/^/  /' || true
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "rmw: ${RMW_IMPLEMENTATION:-default}"
  echo "ros_domain_id: ${ROS_DOMAIN_ID:-0}"
  echo "record_command: ${RECORD_CMD[*]}"
  echo "serial_csv: (opencr_node csv_path — 같은 RUN 이름으로 지정했는지 확인, 예: ~/runs/${RUN}_serial.csv)"
} >"$META/source.txt"

# ── 3. 기록 ──────────────────────────────────────────────────────────────────
echo
echo "기록 시작: $RUN → $BAG  (${DURATION:+${DURATION}초 뒤 자동 종료, }Ctrl+C로 종료)"
echo "  ${RECORD_CMD[*]}"
# 기록 프로세스를 별도 작업(job control)으로 띄우고 Ctrl+C·시간 만료 때 SIGINT를 "한 번만" 전달한다.
# (timeout과 터미널이 함께 SIGINT를 보내면 rosbag2가 두 번 받아 비정상 종료할 수 있음)
set -m
"${RECORD_CMD[@]}" &
RPID=$!
trap 'kill -INT "$RPID" 2>/dev/null || true' INT
TPID=""
if [[ -n "$DURATION" ]]; then ( sleep "$DURATION"; kill -INT "$RPID" 2>/dev/null ) & TPID=$!; fi
set +e
while true; do wait "$RPID"; STATUS=$?; kill -0 "$RPID" 2>/dev/null || break; done
[[ -n "$TPID" ]] && kill "$TPID" 2>/dev/null
set -e
trap - INT
set +m
echo "ros2 bag record 종료 (exit $STATUS)"

# ── 4. 기록 후 확인 ──────────────────────────────────────────────────────────
[[ -d "$BAG" ]] || die "bag 폴더가 만들어지지 않음 — 위 ros2 bag record 오류 확인"
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  warn "metadata.yaml 없음 → ros2 bag reindex 시도"
  ros2 bag reindex "$BAG" -s "$STORAGE" || warn "reindex 실패 — 이 기록은 다시 찍는다"
fi
ros2 bag info "$BAG" >"$META/info.txt" 2>&1 || warn "ros2 bag info 실패 (info.txt 확인)"
set +e
python3 "$TOOL" check "$BAG" | tee "$META/check.txt"
CHECK=${PIPESTATUS[0]}
set -e
(cd "$BAG" && find . -maxdepth 1 -type f -printf '%f\n' | sort | xargs -d '\n' sha256sum) >"$META/SHA256SUMS"
SIZE=$(du -sb "$BAG" | cut -f1)
echo "bag_size_bytes: $SIZE" >>"$META/source.txt"
if [[ $ARCHIVE -eq 1 ]]; then
  echo "압축: $OUT_DIR/$RUN.tar.gz"
  tar -czf "$OUT_DIR/$RUN.tar.gz" -C "$OUT_DIR" "$RUN"
  (cd "$OUT_DIR" && sha256sum "$RUN.tar.gz") >"$META/ARCHIVE.sha256"
fi

cat <<EOF

완료: $RUN  (bag $((SIZE / 1000000)) MB, 확인 결과 $([[ $CHECK -eq 0 ]] && echo PASS || echo FAIL — check.txt 참고))
  bag              $BAG   ← Git에 올리지 않음. $([[ $ARCHIVE -eq 1 ]] && echo "$RUN.tar.gz를" || echo "bag 폴더를 압축해") 팀 공유 드라이브에 올리고 평가자 접근 권한 확인
  증거(Git)         recordings/$RUN/  (source.txt, info.txt, check.txt, SHA256SUMS, params/, config/)
다음 할 일
  1. tracking_logger(Ctrl+C) → results/logs/verification/$RUN.csv 확인
  2. 시리얼 로그 복사: cp ~/runs/${RUN}_serial.csv $LV2/results/logs/verification/
  3. recordings/README.md 3절 표에 1행 추가 (링크·기간·메시지 수·commit·크기·sha256)
  4. 재현은 기록하지 않은 팀원이: tools/bag_replay.sh reanalysis|reprocess <bag>
EOF
exit $CHECK
