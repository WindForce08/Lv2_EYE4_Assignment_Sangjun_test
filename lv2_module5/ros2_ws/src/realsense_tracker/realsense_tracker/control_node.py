"""제어 노드 (담당: 제어) — /target → Pan/Tilt 속도 명령. 시리얼·모터 버스에 접근하지 않음

실행 위치: Raspberry Pi (perception_node·opencr_node와 같은 ROS 2 runtime)
구독
  /target                 geometry_msgs/PointStamped  QoS best effort, depth 1 (perception 발행과 호환)
  /opencr/limit           std_msgs/String "pan:+1"  보드 경계 정지 (opencr_node 발행, reliable depth 10)
                          → 탐색 반환점 / 추적 중이면 범위 초과로 판단 (control_core.on_limit)
발행
  /control/pan_tilt_cmd   realsense_tracker_interfaces/PanTiltCommand  20 Hz + LOST 전환 즉시
                          header.stamp = 명령 생성 시각, 단위 rad/s (모터 원시 부호, direction 적용 후)
                          stop=true 이면 두 축 0
  /tracking_status        std_msgs/String  IDLE / TRACKING / LOST / SEARCHING (명령과 같은 주기)
서비스
  /control/enable         std_srvs/SetBool  false = 명시적 중지·탐색 취소(IDLE, 정지), true = 재개(3프레임 복귀 필요)

판단 로직은 control_core.Controller에 있다. 파라미터: config/control.yaml (실제 추적),
config/control_dry.yaml (모의 시험).
"""
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from realsense_tracker_interfaces.msg import PanTiltCommand

from .control_core import Controller

# 발제문 /target QoS: best effort, depth 1. reliable로 구독하면 best effort 발행과 연결되지 않는다.
TARGET_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
COMMAND_PERIOD_SEC = 0.05  # 20 Hz. opencr_node의 명령 신선도 한계(0.15초)보다 충분히 짧아야 함

PARAM_DEFAULTS = {
    'kp_pan': 0.1, 'kp_tilt': 0.1,
    'pan_speed_limit_rad_s': 0.05, 'tilt_speed_limit_rad_s': 0.05,
    'pan_deadband': 0.03, 'tilt_deadband': 0.03,
    'target_timeout_sec': 0.5, 'recovery_frames': 3,
    'pan_direction': -1, 'tilt_direction': 1,
    # 목표가 없을 때 탐색 (기본 꺼짐 — control.yaml에서 켬, 모의 시험 control_dry.yaml은 끔)
    'search_enabled': False,
    'search_delay_sec': 3.0,          # LOST(정지)로 기다린 뒤 탐색 시작 — 2 s 가림 후 시야 내 재등장 시험을 위해
    'search_speed_rad_s': 0.4,
    'search_tilt_step_rad': 0.5,      # 한 줄 훑은 뒤 Tilt 이동량 (수직 화각 약 0.73 rad보다 작게)
    'search_leg_timeout_sec': 20.0,   # 경계 이벤트 없이 한 방향으로 움직이는 최대 시간 (DRY 등)
    'search_timeout_sec': 120.0,      # 탐색 전체 상한 → IDLE
    'track_stable_sec': 5.0,          # 이 시간 이상 범위 안에서 추적하면 경계 재탐색 기회 초기화
}


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        for key, value in PARAM_DEFAULTS.items():
            self.declare_parameter(key, value)

        def p(key):
            return self.get_parameter(key).value

        self.core = Controller(
            kp_pan=p('kp_pan'), kp_tilt=p('kp_tilt'),
            pan_speed_limit=p('pan_speed_limit_rad_s'), tilt_speed_limit=p('tilt_speed_limit_rad_s'),
            pan_deadband=p('pan_deadband'), tilt_deadband=p('tilt_deadband'),
            timeout=p('target_timeout_sec'), recovery_frames=p('recovery_frames'),
            pan_direction=p('pan_direction'), tilt_direction=p('tilt_direction'),
            search_enabled=p('search_enabled'), search_delay=p('search_delay_sec'),
            search_speed=p('search_speed_rad_s'), search_tilt_step=p('search_tilt_step_rad'),
            search_leg_timeout=p('search_leg_timeout_sec'), search_timeout=p('search_timeout_sec'),
            track_stable=p('track_stable_sec'))
        # depth 1: 오래된 명령이 큐에 쌓여 뒤늦게 전달되지 않게 한다.
        self.cmd = self.create_publisher(PanTiltCommand, '/control/pan_tilt_cmd', 1)
        self.status = self.create_publisher(String, '/tracking_status', 1)
        self.sub = self.create_subscription(PointStamped, '/target', self.target, TARGET_QOS)
        self.limit_sub = self.create_subscription(String, '/opencr/limit', self.limit, 10)
        self.last_state = self.core.state
        self.enable_service = self.create_service(SetBool, '/control/enable', self.enable)
        self.timer = self.create_timer(COMMAND_PERIOD_SEC, self.publish_output)
        self.get_logger().info(
            f"control_node: kp_pan={p('kp_pan')} kp_tilt={p('kp_tilt')} rad/s, "
            f"limit pan/tilt={p('pan_speed_limit_rad_s')}/{p('tilt_speed_limit_rad_s')} rad/s, "
            f"deadband pan/tilt={p('pan_deadband')}/{p('tilt_deadband')}, "
            f"direction pan/tilt={p('pan_direction')}/{p('tilt_direction')}, "
            f"timeout={p('target_timeout_sec')} s, recovery={p('recovery_frames')} frames, "
            f"search={'on' if p('search_enabled') else 'off'} (delay {p('search_delay_sec')} s, "
            f"speed {p('search_speed_rad_s')} rad/s, timeout {p('search_timeout_sec')} s)")

    def target(self, msg):
        previous_state = self.core.state
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.core.receive(msg.point.x, msg.point.y, msg.point.z, stamp,
                          self.get_clock().now().nanoseconds, time.monotonic())
        # 정상 추적은 20 Hz로만 발행. LOST 전환은 다음 타이머를 기다리지 않고 즉시 STOP을 보낸다.
        if previous_state != 'LOST' and self.core.state == 'LOST':
            self.publish_output()

    def limit(self, msg):
        """보드 경계 정지 "pan:+1". 형식이 다르면 무시(로그)."""
        try:
            axis, sign = msg.data.split(':')
            sign = int(sign)
        except ValueError:
            self.get_logger().warning(f'ignored /opencr/limit "{msg.data}"')
            return
        self.core.on_limit(axis, sign, time.monotonic())
        self.get_logger().info(f'board limit {axis}:{sign:+d} → state {self.core.state}')
        self.publish_output()

    def enable(self, request, response):
        self.core.set_enabled(request.data)
        self.publish_output()
        response.success = True
        response.message = ('tracking enabled; 3 fresh frames required' if request.data
                            else 'tracking disabled; IDLE and stop')
        self.get_logger().info(response.message)
        return response

    def publish_output(self):
        now = self.get_clock().now()
        stop, pan, tilt = self.core.output(time.monotonic(), now.nanoseconds)
        msg = PanTiltCommand()
        msg.header.stamp = now.to_msg()
        msg.stop = stop
        msg.pan_velocity_rad_s = pan
        msg.tilt_velocity_rad_s = tilt
        self.cmd.publish(msg)
        state = String()
        state.data = self.core.state
        self.status.publish(state)
        if self.core.state != self.last_state:
            reason = f' ({self.core.idle_reason})' if self.core.state == 'IDLE' else ''
            self.get_logger().info(f'state {self.last_state} → {self.core.state}{reason}')
            self.last_state = self.core.state


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
