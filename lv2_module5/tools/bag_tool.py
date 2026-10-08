"""ROS 2 bag 도우미 (담당: 통합·검증) — 문제 5. ROS 없이 실행된다 (PyYAML만 필요)

  topics  config/bag.yaml에서 기록·재생할 토픽 이름을 출력 (카메라 토픽은 camera.yaml에서 읽음)
  check   bag 폴더의 metadata.yaml로 파일 누락·기간·토픽별 메시지 수·타입을 확인
  compare 두 tracking_logger CSV를 원본 영상 시각(stamp_ns)으로 맞춰 프레임 단위 대조

실행 (lv2_module5 폴더에서)
  python3 tools/bag_tool.py topics record                # 기록 토픽 (bag_record.sh가 사용)
  python3 tools/bag_tool.py topics record --required     # 필수 토픽만
  python3 tools/bag_tool.py topics reprocess             # A 입력 재처리에서 재생할 토픽
  python3 tools/bag_tool.py check ~/bags/success_01
  python3 tools/bag_tool.py compare results/logs/verification/success_01_reanalysis.csv \
                                    results/logs/verification/success_01_reprocess.csv
종료 코드: 0 = 통과, 1 = 실패(파일 누락·필수 토픽 없음 / 대조 값 불일치), 2 = 사용 오류

compare가 가능한 이유: perception_node는 /target header에 입력 영상 header를 그대로 복사한다. 그래서 같은 영상에서 나온
원본 /target과 재처리 /target_replay는 같은 stamp_ns를 가진다. 같은 코드·설정이면 검출기는 결정적이므로 값이 같아야 한다.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import yaml

LV2 = Path(__file__).resolve().parents[1]
BAG_CONFIG = LV2 / "config" / "bag.yaml"
MODES = ("record", "reprocess", "reanalysis")
COMPARE_FIELDS = ("detected", "ex", "ey", "area_ratio")


def load_config(path=BAG_CONFIG):
    """bag.yaml을 읽고 camera.yaml 키를 실제 토픽 이름으로 바꾼다. 반환: (설정 dict, 토픽 목록)"""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    cam_path = (path.parent / cfg["camera_config"]).resolve()
    with open(cam_path, encoding="utf-8") as f:
        cam = yaml.safe_load(f) or {}
    topics = []
    for t in cfg["topics"]:
        name = str(cam.get(t["camera"]) or "").strip() if "camera" in t else t["name"]
        if not name:
            raise SystemExit(f"{cam_path}의 {t['camera']} 값이 비어 있음 — camera.yaml에 실제 토픽 이름을 기록하세요")
        topics.append({**t, "name": name})
    return cfg, topics


def cmd_topics(args):
    _, topics = load_config(args.config)
    names = [t["name"] for t in topics if args.mode in t["use"] and (t["required"] or not args.required)]
    print(" ".join(names))
    return 0


def read_metadata(bag):
    meta = Path(bag) / "metadata.yaml"
    if not meta.is_file():
        return None
    with open(meta, encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information")


def cmd_check(args):
    cfg, topics = load_config(args.config)
    bag = Path(args.bag).expanduser()
    info = read_metadata(bag)
    fails, warns = [], []
    if info is None:
        print(f"FAIL {bag}/metadata.yaml 없음 — 기록이 정상 종료되지 않았을 수 있음. `ros2 bag reindex {bag}` 시도 후 다시 확인")
        return 1
    files = info.get("relative_file_paths") or []
    print(f"bag: {bag}")
    print(f"  저장 형식 {info.get('storage_identifier')}, ROS {info.get('ros_distro', '?')}, 파일 {len(files)}개")
    total = 0
    for name in files:
        p = bag / name
        if p.is_file():
            total += p.stat().st_size
            print(f"    {name}  {p.stat().st_size / 1e6:.1f} MB")
        else:
            fails.append(f"데이터 파일 없음: {name}")
    duration = info["duration"]["nanoseconds"] / 1e9
    lo, hi = cfg["duration_sec"]["min"], cfg["duration_sec"]["max"]
    print(f"  기간 {duration:.2f} s (발제 기준 {lo}~{hi} s), 메시지 {info.get('message_count')}, 합계 {total / 1e6:.1f} MB")
    if not lo <= duration <= hi:
        warns.append(f"기간 {duration:.1f} s가 {lo}~{hi} s 밖")

    recorded = {t["topic_metadata"]["name"]: t for t in info.get("topics_with_message_count") or []}
    ratio_min = cfg["min_rate_ratio"]
    print(f"  {'토픽':40s} {'개수':>7s} {'Hz':>7s} {'기대 Hz':>7s}  판정")
    for t in topics:
        if "record" not in t["use"]:
            continue
        r = recorded.get(t["name"])
        count = r["message_count"] if r else 0
        rate = count / duration if duration > 0 else 0.0
        verdict = "OK"
        if r is None or count == 0:
            verdict = "FAIL 없음" if t["required"] else "WARN 없음(선택)"
            (fails if t["required"] else warns).append(f"{t['name']} 기록 안 됨")
        elif r["topic_metadata"]["type"] != t["type"]:
            verdict = f"FAIL 타입 {r['topic_metadata']['type']}"
            fails.append(f"{t['name']} 타입이 {t['type']}가 아님")
        elif rate < t["rate_hz"] * ratio_min:
            verdict = "WARN 적음"
            warns.append(f"{t['name']} {rate:.1f} Hz < 기대 {t['rate_hz']} Hz × {ratio_min}")
        print(f"  {t['name']:40s} {count:7d} {rate:7.1f} {t['rate_hz']:7d}  {verdict}")
    extra = sorted(set(recorded) - {t["name"] for t in topics})
    if extra:
        print(f"  설정에 없는 추가 토픽: {', '.join(extra)}")
    for w in warns:
        print(f"WARN {w}")
    for e in fails:
        print(f"FAIL {e}")
    print("결과:", "FAIL" if fails else ("PASS (경고 있음)" if warns else "PASS"))
    return 1 if fails else 0


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    by_stamp = {}
    for r in rows:
        by_stamp[int(r["stamp_ns"])] = r
    if len(by_stamp) != len(rows):
        print(f"WARN {path}: 같은 stamp_ns가 {len(rows) - len(by_stamp)}행 중복 — 마지막 행만 사용")
    return by_stamp


def cmd_compare(args):
    base, target = load_rows(args.base), load_rows(args.target)
    if not base or not target:
        print("FAIL 빈 CSV가 있음")
        return 1
    lo, hi = max(min(base), min(target)), min(max(base), max(target))
    print(f"기준 {args.base}: {len(base)}행")
    print(f"대상 {args.target}: {len(target)}행")
    if lo > hi:
        print("FAIL 두 CSV의 영상 시각 구간이 겹치지 않음 — 다른 run을 비교하고 있지 않은지 확인")
        return 1
    in_range = lambda d: {k for k in d if lo <= k <= hi}
    b, t = in_range(base), in_range(target)
    common = sorted(b & t)
    print(f"공통 구간 {(hi - lo) / 1e9:.2f} s: 공통 프레임 {len(common)}, 구간 안에서 기준에만 {len(b - t)}, 대상에만 {len(t - b)} "
          f"(구간 밖 제외: 기준 {len(base) - len(b)}, 대상 {len(target) - len(t)})")
    fields = list(COMPARE_FIELDS) + (["state"] if args.state else [])
    mismatched = 0
    for field in fields:
        diffs, bad = [], []
        for k in common:
            a, c = base[k][field], target[k][field]
            if field in ("detected", "state"):
                if a != c:
                    bad.append(k)
                continue
            d = abs(float(a) - float(c))
            diffs.append(d)
            if d > args.tol or math.isnan(d):
                bad.append(k)
        extra = f", 최대 차 {max(diffs, default=0.0):.6f}" if diffs else ""
        print(f"  {field:11s} 일치 {len(common) - len(bad)}/{len(common)}{extra}")
        for k in bad[:args.show]:
            print(f"      stamp_ns {k}: 기준 {base[k][field]} / 대상 {target[k][field]}")
        if field != "state":
            mismatched += len(bad)
        elif common and len(bad) > 0.1 * len(common):
            print("WARN state 불일치가 10 %를 넘음 — 전환 직후 차이로 보기 어려움. /tracking_status가 재생·기록됐는지 확인")
    if args.state:
        print("  (state는 '수신 순간의 최신 상태'라 LOST↔TRACKING 전환 직후 1~2프레임 차이는 정상 — 판정에서 제외)")
    missing = len(b - t) + len(t - b)
    if mismatched:
        print(f"결과: FAIL — 공통 프레임 값 불일치 {mismatched}건 (설정·코드가 기록 당시와 같은지 확인)")
        return 1
    print("결과: PASS" + (f" (구간 안 빠진 프레임 {missing}개 — 개수와 원인을 기록)" if missing else ""))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(BAG_CONFIG), help="bag 설정 (기본 config/bag.yaml)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("topics", help="기록·재생 토픽 이름 출력")
    p.add_argument("mode", choices=MODES)
    p.add_argument("--required", action="store_true", help="필수 토픽만")
    p.set_defaults(func=cmd_topics)
    p = sub.add_parser("check", help="bag 폴더 확인")
    p.add_argument("bag")
    p.set_defaults(func=cmd_check)
    p = sub.add_parser("compare", help="tracking_logger CSV 두 개를 stamp_ns로 대조")
    p.add_argument("base", help="기준 CSV (B 비교: 실시간 <RUN>.csv, A 비교: <RUN>_reanalysis.csv)")
    p.add_argument("target", help="대상 CSV (<RUN>_reanalysis.csv 또는 <RUN>_reprocess.csv)")
    p.add_argument("--tol", type=float, default=0.0, help="ex·ey·area_ratio 허용 차 (기본 0 = 완전히 같음)")
    p.add_argument("--state", action="store_true", help="state 열도 대조 (B ↔ 실시간 비교용, 판정에는 미포함)")
    p.add_argument("--show", type=int, default=5, help="필드별로 보여 줄 불일치 예시 수")
    p.set_defaults(func=cmd_compare)
    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
