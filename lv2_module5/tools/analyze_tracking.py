"""추적 지표 계산 (담당: 검증) — tracking_logger.py CSV → FPS·RMSE·유효 추적 비율·소실/복귀 구간

실시간 시험 CSV와 bag 재분석 CSV에 같은 산식을 쓴다 (문제 4 성능표 ↔ 문제 5 결과 재분석 비교).
ROS 없이 실행:  python3 tools/analyze_tracking.py results/logs/verification/<run_id>.csv [--plot] [--append-metrics]

산식 (발제문 문제 4, config/test.yaml과 같은 정의)
  처리 FPS        = /target 프레임 수 / (마지막 - 첫 영상 시각)   — perception이 처리한 프레임마다 1번 발행하므로
  노드 검출 비율   = detected 프레임 / 전체   (사람 대조 검출률이 아님 — 그 값은 tools/eval_score.py)
  유효 추적 비율   = (detected 이면서 state==TRACKING) / 전체 프레임
  수평 RMSE       = sqrt(mean(ex²)), 수직 RMSE = sqrt(mean(ey²))  — 유효 추적 프레임만, 제외 수 병기
  소실/복귀 구간   = TRACKING이 끊긴 시각 → 첫 재검출 시각(검출 기준 재등장) → TRACKING 복귀 시각
                    복구 시간(검출 기준) = TRACKING 복귀 - 첫 재검출. 발제문의 "재등장 시각"은 사람이 영상으로
                    판정해 results/logs/verification/recovery_trials.csv에 적는다. 복귀하지 못한 구간은 0초가 아니라 실패.
"""
import argparse
import csv
import math
from pathlib import Path

LV2 = Path(__file__).resolve().parents[1]
METRICS = LV2 / "results" / "metrics.csv"
METRIC_FIELDS = ["run_id", "problem", "metric", "value", "unit", "sample_count", "source", "note"]


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["t"] = float(r["time_s"])
        r["det"] = r["detected"] == "1"
        r["ex"], r["ey"] = float(r["ex"]), float(r["ey"])
    return rows


def lost_events(rows, success_sec=3.0):
    """TRACKING이 끊긴 구간마다 (끊긴 시각, 첫 재검출 시각, TRACKING 복귀 시각, 복구 초, 성공 여부)"""
    events, cur = [], None
    prev_tracking = False
    for r in rows:
        tracking = r["state"] == "TRACKING"
        if prev_tracking and not tracking:
            cur = {"lost_at": r["t"], "redetect_at": None, "tracking_at": None}
        elif cur is not None:
            if cur["redetect_at"] is None and r["det"]:
                cur["redetect_at"] = r["t"]
            if tracking:
                cur["tracking_at"] = r["t"]
                events.append(cur)
                cur = None
        prev_tracking = tracking
    if cur is not None:
        events.append(cur)  # 기록이 끝날 때까지 복귀하지 못함
    for e in events:
        e["recovery_sec"] = (None if e["tracking_at"] is None or e["redetect_at"] is None
                             else e["tracking_at"] - e["redetect_at"])
        e["success"] = e["recovery_sec"] is not None and e["recovery_sec"] <= success_sec
    return events


def summarize(rows):
    n = len(rows)
    duration = rows[-1]["t"] - rows[0]["t"] if n > 1 else 0.0
    valid = [r for r in rows if r["det"] and r["state"] == "TRACKING"]
    rmse = (lambda key: math.sqrt(sum(r[key] ** 2 for r in valid) / len(valid)) if valid else float("nan"))
    return {
        "frames": n, "duration_s": duration,
        "fps": (n - 1) / duration if duration > 0 else float("nan"),
        "node_detected_ratio": sum(r["det"] for r in rows) / n if n else float("nan"),
        "valid_tracking_ratio": len(valid) / n if n else float("nan"),
        "rmse_ex": rmse("ex"), "rmse_ey": rmse("ey"),
        "rmse_samples": len(valid), "rmse_excluded": n - len(valid),
    }


def plot(rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rows]
    num = lambda v: float(v) if v not in ("", None) else float("nan")
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    ax[0].plot(t, [r["ex"] if r["det"] else float("nan") for r in rows], label="ex")
    ax[0].plot(t, [r["ey"] if r["det"] else float("nan") for r in rows], label="ey")
    ax[0].set_ylabel("normalized error"); ax[0].legend(); ax[0].grid(True)
    ax[1].plot(t, [num(r["pan_command"]) for r in rows], label="pan")
    ax[1].plot(t, [num(r["tilt_command"]) for r in rows], label="tilt")
    ax[1].set_ylabel("command [rad/s]"); ax[1].legend(); ax[1].grid(True)
    states = {"IDLE": 0, "LOST": 1, "SEARCHING": 2, "TRACKING": 3}
    ax[2].step(t, [states.get(r["state"], float("nan")) for r in rows], where="post")
    ax[2].set_yticks(list(states.values()), list(states.keys())); ax[2].set_xlabel("time [s]"); ax[2].grid(True)
    fig.suptitle(rows[0]["run_id"] + " (command values are not measured motor motion)")
    fig.tight_layout(); fig.savefig(out, dpi=120)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--plot", action="store_true", help="results/plots/<run_id>.png 저장 (matplotlib 필요)")
    ap.add_argument("--append-metrics", action="store_true", help="results/metrics.csv에 요약 행 추가")
    ap.add_argument("--problem", default="P4", help="metrics.csv의 problem 열 (P3, P4, P5 등)")
    args = ap.parse_args()
    for path in args.csv:
        rows = load(path)
        if not rows:
            print(f"{path}: 빈 파일")
            continue
        run_id, s = rows[0]["run_id"], summarize(rows)
        print(f"\n== {run_id} ({path})")
        print(f"  프레임 {s['frames']}, 기간 {s['duration_s']:.2f} s, 처리 FPS {s['fps']:.2f}")
        print(f"  노드 검출 비율 {s['node_detected_ratio'] * 100:.1f} % (사람 대조 검출률 아님)")
        print(f"  유효 추적 비율 {s['valid_tracking_ratio'] * 100:.1f} % (detected & TRACKING / 전체)")
        print(f"  수평 RMSE {s['rmse_ex']:.4f}, 수직 RMSE {s['rmse_ey']:.4f} "
              f"(포함 {s['rmse_samples']}, 제외 {s['rmse_excluded']})")
        events = lost_events(rows)
        for i, e in enumerate(events, 1):
            fmt = lambda v: "-" if v is None else f"{v:.3f}"
            print(f"  소실 {i}: 끊김 {fmt(e['lost_at'])} s, 첫 재검출 {fmt(e['redetect_at'])} s, "
                  f"TRACKING {fmt(e['tracking_at'])} s, 복구(검출 기준) {fmt(e['recovery_sec'])} s, "
                  f"{'성공' if e['success'] else '실패/미복귀'}")
        if args.plot:
            out = LV2 / "results" / "plots" / f"{run_id}.png"
            print(f"  그래프: {plot(rows, out)}")
        if args.append_metrics:
            new = not METRICS.exists() or METRICS.stat().st_size == 0
            with open(METRICS, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=METRIC_FIELDS, lineterminator="\n")
                if new:
                    w.writeheader()
                src = Path(path).resolve().relative_to(LV2).as_posix() if Path(path).resolve().is_relative_to(LV2) else path
                for metric, value, unit in [("processing_fps", s["fps"], "fps"),
                                            ("valid_tracking_ratio", s["valid_tracking_ratio"] * 100, "%"),
                                            ("rmse_ex", s["rmse_ex"], "normalized"),
                                            ("rmse_ey", s["rmse_ey"], "normalized")]:
                    w.writerow({"run_id": run_id, "problem": args.problem, "metric": metric,
                                "value": f"{value:.4f}", "unit": unit,
                                "sample_count": s["rmse_samples"] if metric.startswith("rmse") else s["frames"],
                                "source": src, "note": "analyze_tracking.py"})
            print(f"  metrics.csv에 추가: {METRICS}")


if __name__ == "__main__":
    main()
