"""OpenCR bridge 노드 (담당: 통합 + 제어) — /control/pan_tilt_cmd → USB 시리얼 → OpenCR

실행 위치: Raspberry Pi (OpenCR USB 연결). 같은 Pi의 control_node와 ROS 2 토픽으로 연결된다.

실행 모드 (설정이 서로 맞지 않으면 시작을 거부한다 — 기본값은 항상 모터 출력 없음)
  | 모드         | serial_mode | dry_run | expected_board_mode | enable_live_hardware | 설정 파일         |
  | DRY sink     | false       | true    | (무시)              | false                | control_dry.yaml  |
  | 시리얼 DRY   | true        | true    | DRY                 | false                | serial_dry.yaml   |
  | 시리얼 LIVE  | true        | false   | LIVE                | true                 | opencr_live.yaml  |
  - DRY sink: 포트를 열지 않고 명령 검사 결과만 /opencr/dry_status로 발행 (dry_bridge.py)
  - 시리얼 DRY: MODE=DRY 펌웨어만 허용 (펌웨어가 모터 버스를 호출하지 않음)
  - 시리얼 LIVE: MODE=LIVE 펌웨어만 허용, 실제 모터 구동

LIVE에서도 자동 동작은 없다
  시작 시 STATUS만 보낸다. 토크 ON(prepare)과 구동 허가(arm)는 운영자가 서비스를 직접 호출해야 한다.
  /opencr/prepare   std_srvs/Trigger  CHECK → HOLD (중립 자세 확인 후 영속도로 토크 ON)
  /opencr/arm       std_srvs/Trigger  READY + 신선한 stop=false 명령일 때 ARM
  /opencr/disarm    std_srvs/Trigger  영속도 + 구동 허가 해제 (토크 유지 — Tilt 낙하 방지)
  /opencr/bridge_status std_msgs/String(JSON)  phase, 최초 FAULT 원인, 보드 STATE 캐시, 경계 정지 횟수
  /opencr/limit     std_msgs/String  보드 경계 정지 1건마다 "pan:+1" 형식 (축:막힌 원시 방향). ARM은 유지된다.
                    control_node가 받아 탐색 방향을 바꾸거나 추적 중 범위 초과로 판단한다 (reliable, depth 10)
  FAULT 후 자동 재ARM·자동 재연결 없음. 노드 종료 시 DISARM을 한 번 보낸다.
  토크 해제(SUPPORTED_OFF)는 카메라를 손으로 지지한 뒤 별도 절차로만 한다 (README 참고).

시리얼 로그 CSV(csv_path)는 bag과 같은 run_id를 파일명에 넣어 연결한다.
"""
import csv
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from realsense_tracker_interfaces.msg import PanTiltCommand

from .dry_bridge import DryBridge
from .serial_core import PosixSerial, SerialBridge, check_mode

USB_BAUDRATE = 115200  # 기본값. 펌웨어 Serial.begin(115200)과 같아야 함 (DYNAMIXEL 버스 1 Mbps와 별개)


class SerialNode(Node):
    def __init__(self):
        super().__init__('opencr_node')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('serial_mode', False)
        self.declare_parameter('expected_board_mode', 'DRY')
        self.declare_parameter('enable_live_hardware', False)
        self.declare_parameter('port', '')
        self.declare_parameter('usb_serial_baudrate', USB_BAUDRATE)
        self.declare_parameter('command_max_age_sec', SerialBridge.COMMAND_AGE)
        self.declare_parameter('csv_path', '')

        def get(key):
            return self.get_parameter(key).value

        if get('serial_mode') is not True:
            raise RuntimeError('SerialNode requires explicit serial_mode=true')
        mode = check_mode(get('dry_run'), get('expected_board_mode'), get('enable_live_hardware'))
        if not get('port'):
            raise RuntimeError('Explicit serial port required (/dev/serial/by-id/...)')
        self.file = None
        self.writer = None
        self.link = None
        self.transport = None
        try:
            if get('csv_path'):
                self.file = open(get('csv_path'), 'x', newline='')  # 'x': 기존 증거를 덮어쓰지 않음
                self.writer = csv.writer(self.file, lineterminator='\n')
                self.writer.writerow(['monotonic_sec', 'direction', 'line'])
            self.status = self.create_publisher(String, '/opencr/bridge_status', 1)
            self.limit_pub = self.create_publisher(String, '/opencr/limit', 10)  # 이벤트는 하나도 잃으면 안 됨
            self.sub = self.create_subscription(PanTiltCommand, '/control/pan_tilt_cmd', self.command, 1)
            self.prepare_service = self.create_service(Trigger, '/opencr/prepare', self.prepare)
            self.arm_service = self.create_service(Trigger, '/opencr/arm', self.arm)
            self.disarm_service = self.create_service(Trigger, '/opencr/disarm', self.disarm)
            self.transport = PosixSerial(get('port'), get('usb_serial_baudrate'))
            self.link = SerialBridge(self.transport, log=self.record, expected_mode=mode,
                                     command_age=float(get('command_max_age_sec')))
            self.timer = self.create_timer(.01, self.tick)
            if mode == 'LIVE':
                self.get_logger().warning('SERIAL LIVE: motor output firmware expected. Only STATUS sent; '
                                       'explicit /opencr/prepare and /opencr/arm required. Support the camera.')
            else:
                self.get_logger().info('SERIAL DRY: MODE=DRY firmware only; explicit prepare and arm required')
        except Exception:
            if self.transport:
                self.transport.close()
            if self.file:
                self.file.close()
            raise

    def record(self, now, direction, line):
        if self.writer:
            self.writer.writerow([now, direction, line])
            self.file.flush()

    def command(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.link.receive_command(msg.stop, float(msg.pan_velocity_rad_s), float(msg.tilt_velocity_rad_s),
                                  stamp, self.get_clock().now().nanoseconds)

    def prepare(self, request, response):
        response.success, response.message = self.link.prepare()
        return response

    def arm(self, request, response):
        response.success, response.message = self.link.arm()
        return response

    def disarm(self, request, response):
        response.success, response.message = self.link.disarm()
        return response

    def tick(self):
        self.link.tick()
        while self.link.limit_events:
            axis, sign = self.link.limit_events.pop(0)
            event = String()
            event.data = f'{axis}:{sign:+d}'
            self.limit_pub.publish(event)
            self.get_logger().info(f'board limit stop {event.data} (ARM kept; resume after stop confirmation)')
        msg = String()
        msg.data = json.dumps(self.link.snapshot())
        self.status.publish(msg)

    def destroy_node(self):
        if self.link:
            self.link.close()
        if self.file:
            self.file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    selector = None
    node = None
    try:
        # 포트를 열 수 있는 노드를 만들기 전에 모드만 먼저 확인한다.
        selector = Node('opencr_node')
        selector.declare_parameter('serial_mode', False)
        use_serial = selector.get_parameter('serial_mode').value
        selector.destroy_node()
        selector = None
        node = SerialNode() if use_serial else DryBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if selector:
            selector.destroy_node()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
