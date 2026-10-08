"""Raspberry Pi ↔ OpenCR USB 시리얼 bridge 상태 기계 (담당: 통합 + 제어) — ROS 없음

역할
  control_node의 PanTiltCommand(rad/s)를 OpenCR 펌웨어(tracking_controller_2axis)의 ASCII 줄 명령
  (STATUS / CHECK / HOLD / ARM / VEL p t / STOP / DISARM)으로 바꾸고, 보드의 STATE 응답을 검사한다.
  opencr_node.py가 10 ms 타이머로 tick()을 호출하고 ROS 명령을 receive_command()로 넘긴다.

DRY / LIVE
  expected_mode='DRY'  : MODE=DRY 펌웨어만 허용 (모터 버스 호출 없음, 피드백은 모의 값)
  expected_mode='LIVE' : MODE=LIVE 펌웨어만 허용 (실제 모터 구동). opencr_node가 명시적 설정
                         (expected_board_mode=LIVE + enable_live_hardware=true)일 때만 이 값을 쓴다.
  보드가 보고한 MODE가 기대와 다르면 즉시 FAULT (명령 전송 없음).

세션 단계 (phase)
  CONNECTING → (STATE BOOT 확인) BOOT
  BOOT --prepare()--> CHECKING → HOLDING → WAIT_HOLD → VERIFY_READY → READY   (토크 ON, 속도 0)
  READY --arm()--> ARMING → VERIFY_ARM → ARMED   (신선한 stop=false 명령이 있을 때만, 자동 ARM 없음)
  ARMED: 신선한 명령마다 VEL 또는 STOP 1회 전송. stop=true는 STOP(논리적 ARM 유지, 정상 목표 소실용)
  ARMED 중 보드 경계 정지 "EVENT LIMIT STOPPED … AXIS=PAN|TILT DIR=±1": FAULT가 아니다. 보드가 두 축을 멈추고
    ARM을 유지한 것이므로 STOP과 같은 정지 확인 흐름(STATUS로 ARMED·영속도 확인 → VEL 재개)을 탄다.
    이 이벤트가 대기 중인 VEL의 응답이므로 VEL 대기를 해제하고, 경계 직후 도착하는 그 VEL의 "ERR STOPPING" 1건은
    무시한다. 축·방향은 limit_events로 넘겨 opencr_node가 /opencr/limit으로 발행한다 (control_node 탐색 반환점).
  ARMED/READY --disarm()--> DISARMING → WAIT_DISARM → VERIFY_DISARM → READY
  어떤 단계든 이상 → FAULT (래치). 자동 재연결·자동 재ARM 없음. 새 세션은 보드 reset 후 prepare부터.

FAULT가 되는 경우 (모두 시험됨: test/test_serial_core.py)
  - ROS 명령이 COMMAND_AGE(기본 0.15초) 넘게 끊김 / 오래된·역순·범위 밖 명령 (ARM 중)
  - 보드 응답 ACK 미수신, STATUS 0.4초 미수신, 보드 피드백 나이 100 ms 초과, 보드 재부팅
  - 보드 EVENT TIMEOUT / EVENT LIMIT DISARMED(이전 펌웨어) / ARMED가 아닐 때의 EVENT LIMIT / FAULT / ERR,
    형식이 깨진 응답, 시리얼 읽기·쓰기 오류
  FAULT 시 보드가 MODE 확인된 상태면 DISARM을 한 번 시도한다 (best effort, 토크 유지).
  최종 안전은 펌웨어의 300 ms 명령 timeout과 모터 Bus_Watchdog이 보장한다 (bridge가 죽어도 동작).
"""
import math
import os
import re
import time

try:  # Linux/Pi 전용 시리얼 설정. Windows 등에서도 상태 기계 단위 시험은 돌 수 있게 한다.
    import termios
    import tty
    SERIAL_ERRORS = (OSError, termios.error)
except ImportError:  # pragma: no cover - 비 POSIX 환경
    termios = tty = None
    SERIAL_ERRORS = (OSError,)

from .control_core import MAX_VELOCITY_RAD_S

# 펌웨어 RAD_S_PER_UNIT (XM430 Goal_Velocity 1 단위 = 0.229 rpm) — 영속도 전환 판정에만 사용
RAD_S_PER_UNIT = 0.229 * 2 * math.pi / 60
BOARD_MODES = ('DRY', 'LIVE')


def check_mode(dry_run, expected_board_mode, enable_live_hardware):
    """시리얼 모드 설정 조합 검사. 반환: 'DRY' 또는 'LIVE'. 모호한 조합은 RuntimeError."""
    if expected_board_mode == 'DRY':
        if dry_run is not True or enable_live_hardware is not False:
            raise RuntimeError('DRY board requires dry_run=true and enable_live_hardware=false')
        return 'DRY'
    if expected_board_mode == 'LIVE':
        if dry_run is not False or enable_live_hardware is not True:
            raise RuntimeError('LIVE board requires dry_run=false and enable_live_hardware=true '
                               '(explicit hardware confirmation)')
        return 'LIVE'
    raise RuntimeError('expected_board_mode must be DRY or LIVE')


class PosixSerial:
    """Linux/Pi transport: nonblocking 8N1 with exclusive ownership. baudrate = opencr_node usb_serial_baudrate."""
    def __init__(self, path, baudrate=115200):
        if termios is None:raise OSError('POSIX serial (termios) is required; run on Linux/Raspberry Pi')
        speed = getattr(termios, f'B{int(baudrate)}', None)
        if speed is None:raise ValueError(f'Unsupported serial baudrate: {baudrate}')
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            termios.tcgetattr(self.fd)
            if hasattr(termios, 'TIOCEXCL'):
                import fcntl
                fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            attrs[2] |= termios.CS8
            if hasattr(termios, 'CRTSCTS'):attrs[2] &= ~termios.CRTSCTS
            attrs[4] = attrs[5] = speed
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd);self.fd = None;raise

    def read(self):
        try:
            data = os.read(self.fd, 4096)
            if not data:raise OSError('Serial peer closed')
            return data
        except BlockingIOError:return b''

    def write(self, data):
        if os.write(self.fd, data) != len(data):raise OSError('Partial serial write')

    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd = None


LIMIT_PATTERN = re.compile(r'^EVENT LIMIT STOPPED ZERO_REQUESTED(?: AXIS=(PAN|TILT) DIR=([+-]1))?$')

PATTERN = re.compile(
    r'^STATE (BOOT|READY|STOPPING|DISARMED|ARMED|FAULT) MODE=(DRY|LIVE) '
    r'ARMED=([01]) GOAL=(-?\d+),(-?\d+) POS=(-?\d+),(-?\d+) '
    r'VEL=(-?\d+),(-?\d+) TORQUE=([01]),([01]) AGE_MS=(\d+) T_MS=(\d+)$')


def parse_state(line):
    match = PATTERN.fullmatch(line)
    if not match:raise ValueError('Malformed STATE response')
    v = match.groups()
    return dict(state=v[0], mode=v[1], armed=int(v[2]),
                goal=(int(v[3]), int(v[4])), pos=(int(v[5]), int(v[6])),
                vel=(int(v[7]), int(v[8])), torque=(int(v[9]), int(v[10])),
                age_ms=int(v[11]), t_ms=int(v[12]))


class SerialBridge:
    """Tick from a 10 ms timer. Every VEL/STOP uses a new, fresh ROS message."""
    COMMAND_AGE = 0.15     # ROS 명령 최대 나이(초): 원본 시각 기준 + 수신 후 유효 기간
    ACK_TIMEOUT = 0.20     # 명령 1건의 ACK 대기
    STATUS_PERIOD = 0.10   # STATUS 조회 주기 (정지 확인 중에는 0.02초)
    STATUS_MAX_AGE = 0.40  # 이 시간 동안 STATE가 안 오면 FAULT
    STOP_KEEPALIVE = 0.15  # 정지가 느릴 때 STOP 재전송 간격 (펌웨어 명령 timeout 300 ms의 절반)

    def __init__(self, transport, log=None, clock=time.monotonic, expected_mode='DRY', command_age=None):
        if expected_mode not in BOARD_MODES:raise ValueError('expected_mode must be DRY or LIVE')
        if command_age is not None:
            # Pi 내부 DDS 전달·실행 지연 여유. 펌웨어 300 ms timeout보다 짧아야 한다.
            if not 0<command_age<0.3:raise ValueError('command_age must be in (0, 0.3) seconds')
            self.COMMAND_AGE=command_age
        self.expected_mode=expected_mode
        self.transport=transport;self.clock=clock;self.log=log or (lambda *args:None)
        self.phase='CONNECTING';self.reason='';self.board=None;self.mode_verified=False
        self.pending=None;self.buffer=b'';self.last_status=None;self.last_poll=0.0
        self.last_stamp=None;self.latest=None;self.sequence=0;self.sent_sequence=0
        self.stopping=False;self.previous_goal=(0,0);self.closed=False
        self.phase_deadline=None;self.last_command_tx=0.0
        self.limit_events=[]          # (axis 'pan'|'tilt'|'unknown', 원시 방향 +1|-1|0) — opencr_node가 꺼내 발행
        self.limit_count=0;self.last_limit=None
        self.stopping_err_allowed=False  # 경계 정지와 엇갈린 VEL의 ERR STOPPING 1건 허용
        self.send('STATUS', 'STATUS')

    def emit(self, direction, line):self.log(self.clock(), direction, line)

    def send(self, command, expected, timeout=None):
        if self.closed:return False
        if self.pending:raise RuntimeError('Only one outstanding transaction permitted')
        try:self.transport.write((command+'\n').encode('ascii'))
        except SERIAL_ERRORS as exc:
            self.fail('TRANSPORT_WRITE: '+str(exc), try_stop=False);return False
        self.emit('TX',command)
        self.pending=(expected,self.clock()+(timeout or self.ACK_TIMEOUT))
        if expected=='STATUS':self.last_poll=self.clock()
        if expected in ('ARM','VEL','STOP'):self.last_command_tx=self.clock()  # these refresh the board command timer
        return True

    def fail(self, reason, try_stop=True):
        if self.phase=='FAULT':return
        self.phase='FAULT';self.reason=reason;self.pending=None;self.latest=None
        self.emit('FAULT',reason)
        # Best effort once, never repeat a nonzero command or auto-enable torque.
        if try_stop and self.mode_verified and not self.closed:
            try:
                self.transport.write(b'DISARM\n');self.emit('TX','DISARM')
            except SERIAL_ERRORS:pass

    def receive_command(self, stop, pan, tilt, stamp_ns, ros_now_ns):
        """ROS 명령 1건. 원본 시각이 미래/너무 오래됨/역순이거나 속도가 상한 밖이면 거부.
        stamp(control_node)와 ros_now(opencr_node)는 모두 같은 Raspberry Pi의 시계다."""
        age=(ros_now_ns-stamp_ns)/1e9
        valid=all(math.isfinite(v) and abs(v)<=MAX_VELOCITY_RAD_S+1e-8 for v in (pan,tilt))
        ordered=self.last_stamp is None or stamp_ns>self.last_stamp
        if self.phase=='FAULT':return False
        if not valid or stamp_ns<=0 or not 0<=age<=self.COMMAND_AGE or not ordered:
            self.latest=None
            if self.phase in ('ARMING','VERIFY_ARM','ARMED','DISARMING'):self.fail('INVALID_OR_STALE_COMMAND')
            self.emit('REJECT','INVALID_OR_STALE_COMMAND');return False
        self.last_stamp=stamp_ns;self.sequence+=1
        # Account for both receipt age and source age even without newer callbacks.
        deadline=self.clock()+min(self.COMMAND_AGE,self.COMMAND_AGE-age)
        limit=MAX_VELOCITY_RAD_S
        self.latest=(bool(stop),max(-limit,min(limit,pan)),max(-limit,min(limit,tilt)),deadline,self.sequence)
        return True

    def prepare(self):
        """운영자 명시 요청: CHECK(모델·모드·중립 자세 확인) → HOLD(영속도로 토크 ON)."""
        if self.phase!='BOOT':return False,'Requires a fresh BOOT state; reset firmware first'
        # LIVE CHECK blocks the board ~1.25 s (bus init + ping incl. Protocol 1.0 fallback); no STATE during it.
        self.phase='CHECKING';self.phase_deadline=self.clock()+2.5
        self.send('CHECK','CHECK',2.0);return True,'CHECK/HOLD requested; wait for READY status'

    def arm(self):
        if self.phase!='READY' or self.pending:return False,'Requires READY with no outstanding transaction'
        if not self.latest or self.latest[0] or self.clock()>=self.latest[3]:
            return False,'Requires a fresh stop=false ROS command; no automatic ARM'
        self.phase='ARMING';self.phase_deadline=self.clock()+0.4
        self.send('ARM','ARM');return True,'ARM requested; wait for ARMED confirmation'

    def disarm(self):
        if self.phase not in ('ARMED','ARMING','READY'):return False,'Not in a disarmable session'
        # Stop requests preempt outstanding ACKs; never use those ACKs to re-arm.
        self.pending=None;self.phase='DISARMING';self.phase_deadline=self.clock()+0.7
        self.send('DISARM','DISARM');return True,'DISARM requested; torque retained by firmware'

    def on_state(self, line):
        try:board=parse_state(line)
        except ValueError:self.fail('MALFORMED_STATE');return
        if board['mode']!=self.expected_mode:
            self.fail(f"MODE_MISMATCH: board {board['mode']}, expected {self.expected_mode}",try_stop=False);return
        if self.board and board['t_ms']<self.board['t_ms'] and self.board['t_ms']-board['t_ms']<2**31:
            self.fail('BOARD_REBOOT');return
        self.mode_verified=True;self.board=board;self.last_status=self.clock()
        if board['state']=='FAULT':self.fail('BOARD_FAULT: original reason may be unavailable');return
        status_response = bool(self.pending and self.pending[0]=='STATUS')
        if status_response:
            self.pending=None
            self.stopping_err_allowed=False  # 그 VEL의 응답은 이 STATUS 응답보다 먼저 온다
        if self.phase=='CONNECTING':
            if board['state']!='BOOT' or board['armed'] or board['torque']!=(0,0):
                self.fail('STARTUP_NOT_BOOT: reset board, no session adoption');return
            self.phase='BOOT'
        elif self.phase=='VERIFY_READY':
            if board['state']=='DISARMED' and not board['armed'] and board['goal']==(0,0) and board['vel']==(0,0) and board['torque']==(1,1):
                self.phase='READY';self.phase_deadline=None
            else:self.fail('HOLD_NOT_CONFIRMED')
        elif self.phase=='VERIFY_ARM':
            if board['state']=='ARMED' and board['armed'] and board['torque']==(1,1):
                self.phase='ARMED';self.phase_deadline=None
            else:self.fail('ARM_NOT_CONFIRMED')
        elif self.phase=='VERIFY_DISARM':
            if board['state']=='DISARMED' and not board['armed'] and board['goal']==(0,0) and board['vel']==(0,0):
                self.phase='READY';self.phase_deadline=None;self.previous_goal=(0,0)
            else:self.fail('DISARM_NOT_CONFIRMED')
        elif self.phase=='ARMED':
            if not board['armed']:self.fail('BOARD_DISARMED');return
        # A STOPPED event can belong to an earlier STOP. Only a newer,
        # solicited STATUS proves the current stopping cycle has completed.
        if self.phase=='ARMED':
            if board['state']=='STOPPING':self.stopping=True
            elif status_response and board['state']=='ARMED' and board['goal']==(0,0) and board['vel']==(0,0):
                self.stopping=False;self.previous_goal=(0,0)
        if self.phase in ('READY','ARMED') and board['age_ms']>100:
            self.fail('STALE_BOARD_FEEDBACK')

    def on_limit_stop(self, line):
        """보드가 경계에서 두 축을 멈추고 ARM을 유지했다 → STOP과 같은 정지 확인 흐름으로 이어 간다."""
        match=LIMIT_PATTERN.fullmatch(line)
        if not match or self.phase!='ARMED':
            self.fail('BOARD_'+line);return
        axis=match.group(1).lower() if match.group(1) else 'unknown'
        sign=int(match.group(2)) if match.group(2) else 0
        self.limit_count+=1;self.last_limit=(axis,sign)
        self.limit_events.append((axis,sign))
        self.stopping=True;self.previous_goal=(0,0)
        if self.pending and self.pending[0]=='VEL':
            # The board answers the VEL that hit the edge with this event instead of ACK VEL.
            # If the edge came from its periodic check instead, the in-flight VEL is answered with ERR STOPPING.
            self.pending=None;self.stopping_err_allowed=True

    def on_line(self, line):
        self.emit('RX',line)
        if self.phase=='FAULT':return
        if line.startswith('FAULT '):self.fail('BOARD_'+line);return
        if line.startswith('EVENT LIMIT STOPPED'):self.on_limit_stop(line);return
        if line.startswith('EVENT TIMEOUT') or line.startswith('EVENT LIMIT'):
            self.fail('BOARD_'+line);return
        if line.startswith('STATE '):self.on_state(line);return
        if line=='ERR STOPPING' and self.stopping_err_allowed:
            self.stopping_err_allowed=False;self.emit('IGNORED','ERR STOPPING for the VEL crossed by a limit stop');return
        if line.startswith('ERR '):self.fail('BOARD_'+line);return
        if line=='EVENT STOPPED TORQUE_RETAINED':
            # Do not unlock normal VEL here: this event may predate a newer STOP.
            if self.phase=='WAIT_HOLD':
                self.phase='VERIFY_READY';self.pending=None;self.send('STATUS','STATUS')
            elif self.phase=='WAIT_DISARM':
                self.phase='VERIFY_DISARM';self.pending=None;self.send('STATUS','STATUS')
            return
        if not self.pending:return
        expected=self.pending[0]
        if expected=='CHECK' and line=='CHECK_OK BOTH_TORQUES_OFF':
            # CHECK_OK proves the board is responsive; restart the STATUS age clock paused during CHECKING.
            self.last_status=self.clock()
            self.pending=None;self.phase='HOLDING';self.send('HOLD','HOLD');return
        if expected=='HOLD' and line=='ACK HOLD ZERO_REQUESTED':
            self.pending=None;self.phase='WAIT_HOLD';self.phase_deadline=self.clock()+0.65;return
        if expected=='ARM' and line=='ACK ARM':
            self.pending=None;self.phase='VERIFY_ARM';self.send('STATUS','STATUS');return
        if expected=='DISARM' and line=='ACK DISARM ZERO_REQUESTED':
            self.pending=None;self.phase='WAIT_DISARM';return
        if expected=='VEL' and line=='ACK VEL':self.pending=None;return
        if expected=='STOP' and line=='ACK STOP ZERO_REQUESTED':
            self.stopping=True;self.pending=None;return
        if line.startswith('ACK ') or line.startswith('CHECK_OK '):
            self.emit('IGNORED','Unmatched acknowledgement; no state advance')

    def tick(self):
        if self.closed or self.phase=='FAULT':return
        try:data=self.transport.read()
        except SERIAL_ERRORS as exc:self.fail('TRANSPORT_READ: '+str(exc));return
        self.buffer+=data
        if len(self.buffer)>4096:self.fail('SERIAL_BUFFER_OVERFLOW');return
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            if len(line)>512:self.fail('SERIAL_LINE_OVERFLOW');return
            try:text=line.rstrip(b'\r').decode('ascii')
            except UnicodeDecodeError:self.fail('NON_ASCII_SERIAL');return
            self.on_line(text)
            if self.phase=='FAULT':return
        now=self.clock()
        if self.phase_deadline and now>=self.phase_deadline:self.fail('SESSION_TRANSITION_TIMEOUT');return
        if self.phase in ('ARMING','VERIFY_ARM','ARMED'):
            if not self.latest or now>=self.latest[3]:self.fail('ROS_COMMAND_TIMEOUT');return
        if self.phase!='CHECKING' and self.last_status is not None and now-self.last_status>=self.STATUS_MAX_AGE:
            self.fail('STATUS_TIMEOUT');return
        if self.pending:
            if now>=self.pending[1]:self.fail('RESPONSE_TIMEOUT: '+self.pending[0])
            return
        if self.phase in ('WAIT_HOLD','WAIT_DISARM'):return
        if self.mode_verified and now-self.last_poll>=(0.02 if self.stopping else self.STATUS_PERIOD):
            self.send('STATUS','STATUS');return
        if self.phase=='ARMED' and self.latest and self.latest[4]!=self.sent_sequence:
            stop,pan,tilt,deadline,sequence=self.latest
            if now>=deadline:self.fail('ROS_COMMAND_TIMEOUT');return
            self.sent_sequence=sequence
            if self.stopping:
                # Never send VEL until completion is confirmed (STATUS polled above).
                # A STOP after the board finished would restart its completion window, so repeat STOP
                # only while the board still reports STOPPING (firmware keeps stopAt/quiet then) and the
                # board's 300 ms command timer would otherwise expire during a slow stop.
                if (self.board and self.board['state']=='STOPPING'
                        and now-self.last_command_tx>=self.STOP_KEEPALIVE):
                    self.send('STOP','STOP')
                return
            if stop:
                self.stopping=True;self.send('STOP','STOP')
            else:
                # 펌웨어와 같은 반올림으로 원시 목표 속도를 예측: 0으로 바뀌면 정지 확인 단계로 들어간다.
                next_goal=(int(math.floor(abs(pan)/RAD_S_PER_UNIT+.5))* (1 if pan>=0 else -1),
                           int(math.floor(abs(tilt)/RAD_S_PER_UNIT+.5))* (1 if tilt>=0 else -1))
                if next_goal==(0,0) and self.previous_goal!=(0,0):self.stopping=True
                self.previous_goal=next_goal
                self.send(f'VEL {pan:.6f} {tilt:.6f}','VEL')
            return
        if self.mode_verified and now-self.last_poll>=(0.02 if self.stopping else self.STATUS_PERIOD):
            self.send('STATUS','STATUS')
        if self.phase!='CHECKING' and self.last_status is not None and now-self.last_status>=self.STATUS_MAX_AGE:
            self.fail('STATUS_TIMEOUT')

    def snapshot(self):
        return dict(phase=self.phase,reason=self.reason,serial_opened=not self.closed,
                    expected_mode=self.expected_mode,board=self.board,
                    limit_count=self.limit_count,last_limit=self.last_limit,
                    board_receipt_age_sec=None if self.last_status is None else self.clock()-self.last_status)

    def close(self):
        if self.closed:return
        if self.mode_verified:
            try:self.transport.write(b'DISARM\n');self.emit('TX','DISARM')
            except SERIAL_ERRORS:pass
        self.transport.close();self.closed=True
