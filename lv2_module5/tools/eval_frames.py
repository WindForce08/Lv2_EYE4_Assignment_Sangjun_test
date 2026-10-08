"""검출률·배경 오검출 평가용 프레임 저장 (발제문 문제 4: 목표가 보이는 30프레임 / 목표 없는 10프레임을 사람이 대조)

필요: RealSense wrapper와 perception_node가 실행 중이고, perception_node는 publish_debug_image:=true
정해진 시간 동안 고르게 N장을 골라 원본·검출 결과 이미지를 저장하고, 사람이 판정을 적을 CSV 목록을 만든다.

실행 (lv2_module5 폴더에서, ROS 2 환경 — source /opt/ros/lyrical/setup.bash 와 패키지 install/setup.bash):
  python3 tools/eval_frames.py --scene visible --note "거리 40cm, 실내 조명"   # 목표가 보이는 장면: 기본 30장 / 15초
  python3 tools/eval_frames.py --scene empty   --note "목표 치움"              # 목표가 없는 장면: 기본 10장 / 10초
창에 인지 노드의 검출 화면이 실시간으로 뜸 → 장면을 준비한 뒤 SPACE를 누르면 저장 시작, q(또는 ESC)는 종료
화면 없이 바로 저장하려면 --no-view
저장:
  results/images/evaluation/<scene>_<시각>/NN_raw.png, NN_det.png   원본 / 노드 검출 결과 그림
  results/logs/perception/eval_<scene>_<시각>.csv                    프레임 목록 + 노드 출력 + 사람 판정 칸(human_ok)
  results/logs/perception/eval_runs.csv                              실행 기록 (조건·설정·처리 FPS)
판정: CSV의 human_ok 칸에 1(노드 출력이 맞음) 또는 0(틀림)을 적은 뒤 → python3 tools/eval_score.py <CSV들>
bag 입력 재처리 이미지 (문제 5): --output-dir를 주면 그 폴더에 이미지·CSV를 저장하고 eval_runs.csv에는 쓰지 않는다
  (사람 대조 평가 세트와 섞이지 않게). tools/bag_replay.sh reprocess --images N이 사용한다.
"""
import argparse
import csv
import time
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, TimeSynchronizer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from common import LV2, RESULTS, load_camera, load_params
from realsense_tracker.perception_node import image_to_numpy

DEFAULTS = {"visible": (30, 15.0), "empty": (10, 10.0)}  # 장면: (저장할 장수, 고르게 나눌 기간 초)
WIN = "eval_frames (SPACE: start, q: quit)"  # 창 제목은 영문으로 (시스템 OpenCV(Qt)에서 한글이 깨짐)
FIELDS = ["scene", "idx", "time_s", "stamp", "raw_image", "det_image",
          "node_detected", "ex", "ey", "area_ratio", "human_ok", "note"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, choices=list(DEFAULTS),
                    help="visible = 목표가 보이는 장면, empty = 목표가 없는 장면")
    ap.add_argument("--frames", type=int, help="저장할 장수 (기본 visible 30, empty 10)")
    ap.add_argument("--duration", type=float, help="이 시간(초) 동안 고르게 나눠 저장 (기본 visible 15, empty 10)")
    ap.add_argument("--note", default="", help="시험 조건 메모 (거리, 조명, 배경 등)")
    ap.add_argument("--no-view", action="store_true", help="확인 창 없이 바로 저장 시작")
    ap.add_argument("--target-topic", default="/target",
                    help="검출 결과 토픽 (bag 입력 재처리 이미지를 저장할 때는 /target_replay)")
    ap.add_argument("--output-dir", help="저장 폴더 (bag 재처리 이미지용). 주면 eval_runs.csv에 기록하지 않음")
    args = ap.parse_args()
    view = not args.no_view
    n_frames = args.frames or DEFAULTS[args.scene][0]
    duration = args.duration or DEFAULTS[args.scene][1]
    interval = duration / n_frames

    cam, params = load_camera(), load_params()
    run_id = f"{args.scene}_{time.strftime('%Y%m%d_%H%M%S')}"
    if args.output_dir:
        img_dir = log_dir = Path(args.output_dir).resolve()
    else:
        img_dir = RESULTS / "images" / "evaluation" / run_id
        log_dir = RESULTS / "logs" / "perception"
    rel = lambda p: p.relative_to(LV2) if p.is_relative_to(LV2) else p

    rclpy.init()
    node = rclpy.create_node("eval_frames")
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)  # /target은 best effort 발행
    # 같은 영상(같은 시각)의 원본·검출 그림·/target을 하나로 묶음
    sync = TimeSynchronizer([Subscriber(node, Image, cam["color_topic"], qos_profile=reliable),
                             Subscriber(node, Image, "/perception/debug_image", qos_profile=reliable),
                             Subscriber(node, PointStamped, args.target_topic, qos_profile=best_effort)], queue_size=30)
    count = {"target": 0}  # 저장 구간 동안 받은 /target 수 → 처리 FPS 확인용
    node.create_subscription(PointStamped, args.target_topic, lambda m: count.__setitem__("target", count["target"] + 1),
                             best_effort)

    rows = []
    st = {"recording": not view, "start": None, "next": None, "count0": 0}  # 창이 있으면 SPACE를 눌러야 저장 시작
    live = {"det": None, "first": None}  # 화면에 띄울 최신 검출 그림, 첫 영상을 받은 시각

    def on_frame(color, det, target):
        now = time.monotonic()
        live["det"] = det
        if live["first"] is None:
            live["first"] = now
        if not st["recording"]:
            return
        if st["start"] is None:
            st.update(start=now, next=now, count0=count["target"])
        if len(rows) >= n_frames or now < st["next"]:
            return
        st["next"] += interval                       # 다음 저장 시각 (고르게 나눔)
        idx = len(rows) + 1
        img_dir.mkdir(parents=True, exist_ok=True)
        log_dir.mkdir(parents=True, exist_ok=True)
        raw_path, det_path = img_dir / f"{idx:02d}_raw.png", img_dir / f"{idx:02d}_det.png"
        cv2.imwrite(str(raw_path), image_to_numpy(color))
        cv2.imwrite(str(det_path), image_to_numpy(det))
        p = target.point
        rows.append({"scene": args.scene, "idx": idx, "time_s": f"{now - st['start']:.2f}",
                     "stamp": f"{target.header.stamp.sec}.{target.header.stamp.nanosec:09d}",
                     "raw_image": str(rel(raw_path)), "det_image": str(rel(det_path)),
                     "node_detected": int(p.z > 0), "ex": f"{p.x:.4f}", "ey": f"{p.y:.4f}",
                     "area_ratio": f"{p.z:.5f}", "human_ok": "", "note": ""})
        print(f"  {idx:2d}/{n_frames}  t={now - st['start']:5.2f}s  "
              f"{'검출' if p.z > 0 else '미검출'}  ex={p.x:+.3f} ey={p.y:+.3f} z={p.z:.4f}")

    sync.registerCallback(on_frame)
    print(f"[{run_id}] {duration:.0f}초 동안 {n_frames}장을 고르게 저장합니다 ({interval:.2f}초 간격)")
    if view:
        print("창에서 장면을 확인하고 SPACE를 누르면 저장을 시작합니다 (q 또는 ESC: 종료)")
        # 창을 먼저 만들어 둠 — 시스템 OpenCV 4.6(Qt)은 imshow로 바로 띄우면 창이 작게 뜨고 영상이 검게 나옴
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
    t_wait = time.monotonic()
    try:
        while rclpy.ok() and len(rows) < n_frames:
            rclpy.spin_once(node, timeout_sec=0.01 if view else 0.05)
            if view and live["det"] is not None:
                shown = image_to_numpy(live["det"]).copy()
                msg = (f"REC {len(rows)}/{n_frames}" if st["recording"]
                       else f"SPACE: start ({n_frames} frames / {duration:.0f}s)   q: quit")
                color = (0, 0, 255) if st["recording"] else (0, 255, 255)
                cv2.putText(shown, msg, (10, shown.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(shown, msg, (10, shown.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.imshow(WIN, shown)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" ") and not st["recording"]:
                    st["recording"] = True
                    print("저장 시작")
                elif key in (ord("q"), 27):
                    print("종료 — 저장한 프레임까지만 기록합니다")
                    break
            if live["first"] is None and time.monotonic() - t_wait > 10:
                print("10초 동안 영상을 받지 못함 → RealSense wrapper와 perception_node(publish_debug_image:=true)가 "
                      "실행 중인지, 모든 터미널의 ROS_DOMAIN_ID가 같은지 확인하세요")
                break
            if st["start"] is not None and time.monotonic() - st["start"] > duration + 10:
                print("시간 초과 — 저장한 프레임까지만 기록합니다")
                break
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C로 중간에 끄면 저장한 프레임까지만 기록
        print("\n중단됨 — 저장한 프레임까지만 기록합니다")
    if view:
        cv2.destroyAllWindows()
    elapsed = time.monotonic() - st["start"] if st["start"] else 0.0
    target_rate = (count["target"] - st["count0"]) / elapsed if elapsed > 0 else 0.0
    node.destroy_node()
    rclpy.try_shutdown()
    if not rows:
        return

    csv_path = log_dir / f"eval_{run_id}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    if args.output_dir:  # bag 재처리 이미지: 사람 대조 평가 실행 목록(eval_runs.csv)에 넣지 않음
        print(f"\n저장 완료: {len(rows)}장 → {rel(img_dir)} (/target 수신 {target_rate:.1f} Hz)")
        return
    runs_path = log_dir / "eval_runs.csv"
    new = not runs_path.exists()
    with open(runs_path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["run_id", "scene", "frames_saved", "duration_s", "interval_s", "target_rate_hz",
                        "hsv_lower", "hsv_upper", "min_area_ratio", "camera", "note"])
        w.writerow([run_id, args.scene, len(rows), f"{elapsed:.1f}", f"{interval:.2f}", f"{target_rate:.1f}",
                    params["hsv_lower"], params["hsv_upper"], params["min_area_ratio"],
                    f"{cam['width']}x{cam['height']}@{cam['fps']}", args.note])

    print(f"\n저장 완료: {len(rows)}장 → {img_dir.relative_to(LV2)}")
    print(f"판정 목록: {csv_path.relative_to(LV2)}  (human_ok 칸에 1=맞음 / 0=틀림 을 적은 뒤 eval_score.py로 집계)")
    print(f"저장 구간 /target 수신 {target_rate:.1f} Hz (처리 FPS 확인용, 노드 로그의 '처리 FPS'와 함께 기록)")


if __name__ == "__main__":
    main()
