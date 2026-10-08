"""Pan/Tilt 2축 P 제어 로직 (담당: 제어) — ROS·시리얼·모터 접근 없음

역할
  /target 한 프레임(ex, ey, area_ratio, 원본 영상 시각)을 받아 상태(IDLE/TRACKING/LOST/SEARCHING)를
  갱신하고, 현재 상태에서 내보낼 두 축 속도 명령(rad/s)을 계산한다.
  control_node.py가 ROS 입출력을, 이 파일이 판단을 담당하므로 ROS 없이 단위 시험할 수 있다.

입력 규약 (발제문 /target, geometry_msgs/PointStamped)
  x = ex = (cx - W/2)/(W/2)   오른쪽 +   [-1, 1]
  y = ey = (cy - H/2)/(H/2)   아래쪽 +   [-1, 1]
  z = area_ratio              검출되면 (0, 1], 미검출이면 정확히 0
  z == 0 이면 x, y는 의미가 없으므로 제어에 쓰지 않는다.
  보드 경계 정지 on_limit(axis, sign): opencr_node의 /opencr/limit ("pan:+1") — sign은 막힌 원시 방향

출력 (stop, pan_rad_s, tilt_rad_s)
  TRACKING  pan  = clamp(pan_direction  * kp_pan  * ex, ±pan_speed_limit)    (|ex| <= pan_deadband 이면 0)
            tilt = clamp(tilt_direction * kp_tilt * ey, ±tilt_speed_limit)   (|ey| <= tilt_deadband 이면 0)
  SEARCHING 탐색 패턴 (아래) — 원시 부호의 ±search_speed
  그 밖     stop=true, 두 축 0
  direction(+1/-1)은 "영상 오차 부호 → 모터 원시 속도 부호" 변환이며 **이 파일에서 한 번만** 적용한다.
  opencr_node와 OpenCR 펌웨어는 부호를 다시 곱하지 않는다.

상태 전이
  IDLE(start)        시작 상태. 미검출 프레임 → LOST, 신선한 검출 3프레임 → TRACKING
  TRACKING           미검출·무효·오래된 입력·timeout → LOST (즉시 정지, 이전 속도 유지 금지)
                     경계 정지(on_limit) → SEARCHING (범위 초과로 목표를 놓침) — 한 번 다시 찾은 뒤 같은 일이
                     track_stable 안에 또 생기면 목표가 범위 밖에 있는 것 → IDLE(out_of_range)
  LOST               정지. 신선한 검출 3프레임 → TRACKING
                     search_enabled이고 **신선한 미검출 프레임이 계속 오는 동안** search_delay 지나면 → SEARCHING
                     (입력 자체가 끊긴 timeout에서는 탐색하지 않는다 — 카메라 없이 돌지 않음)
  SEARCHING          탐색 패턴 구동. 신선한 검출 3프레임 → TRACKING. timeout·무효 입력 → LOST(정지)
                     패턴 완료 또는 search_timeout → IDLE(search_done) (정지)
  IDLE(search_done)  정지. 미검출은 IDLE 유지(탐색 반복 없음), 신선한 검출 3프레임 → TRACKING
  IDLE(out_of_range) 정지(경계에 멈춘 자세). 범위 밖 방향을 요구하는 검출은 무시, 안쪽으로 돌아온 목표만 → TRACKING
  IDLE(disabled)     set_enabled(False). 입력 무시. set_enabled(True)면 IDLE(start)

탐색 패턴 (search_*): 각도 한계는 위치를 아는 펌웨어만 안다. 여기서는 한 방향으로 움직이다가 보드 경계 정지
  이벤트를 반환점으로 쓴다 (펌웨어는 경계에서 ARM을 유지한 채 멈춘다).
  1 LOCAL     지금 Tilt 높이에서 Pan을 마지막으로 쫓던 방향 경계까지, 이어서 반대쪽 경계까지 (놓친 목표는 대개 같은 높이)
  2 TILT_EDGE Tilt를 마지막으로 쫓던 방향 경계까지 (Pan 정지)
  3 SWEEP     Pan을 반대쪽 경계까지 한 줄 훑음
  4 STEP      Tilt를 search_tilt_step만큼(시간 = step/속도) 반대쪽으로 이동 — 도중에 Tilt 경계면 마지막 줄
  3·4 반복 → Tilt 반대쪽 경계 줄까지 훑으면 완료. 경계 이벤트가 오지 않는 환경(DRY)은 leg timeout으로 진행
  위치를 적분해 한계를 만들지 않는다 (STEP 시간은 패턴 간격일 뿐, 한계 판단은 펌웨어 이벤트)

안전 규칙
  - 미검출(z=0), 값 이상, 오래된/중복/역순 시각 → TRACKING이면 즉시 LOST, 두 축 0 (이전 속도 유지 금지)
  - 마지막 신선한 입력 후 timeout(기본 0.5초) → LOST, 두 축 0 (SEARCHING 중에도)
  - LOST/IDLE/SEARCHING → TRACKING 복귀는 신선한 검출 프레임 recovery_frames(기본 3)개 연속일 때만
  - set_enabled(False) → IDLE (명시적 중지·탐색 취소). 다시 켜도 3프레임 조건을 거쳐야 TRACKING
  - 경계 정지 이후 그 축·방향(edge_block)으로는 명령하지 않는다. 안쪽 명령이 EDGE_RELEASE_SEC 이어지면 해제
    (보드가 정지를 끝내고 실제로 경계에서 벗어난 뒤 — 위치를 모르므로 시간으로 판단)
  - 실제 엔코더 각도 제한은 위치를 아는 OpenCR 펌웨어가 담당한다.
"""
import math

# 펌웨어 tracking_controller_2axis.ino 의 MAX_RAD_S 와 같은 값 (펌웨어가 이보다 큰 VEL을 거부).
# control의 축별 속도 상한·탐색 속도는 이 값 이하여야 하며, opencr_node도 같은 상한으로 명령을 검사한다.
MAX_VELOCITY_RAD_S = 0.5

AXES = ('pan', 'tilt')
EDGE_RELEASE_SEC = 0.5   # 경계 막힘 해제에 필요한 연속 안쪽 명령 시간 (보드 정지 확인 ≈0.2 s + 실제 이동)


def _sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


class Controller:
    def __init__(self, kp_pan=0.1, kp_tilt=0.1,
                 pan_speed_limit=0.05, tilt_speed_limit=0.05,
                 pan_deadband=0.03, tilt_deadband=0.03,
                 timeout=0.5, recovery_frames=3, pan_direction=-1, tilt_direction=1,
                 search_enabled=False, search_delay=3.0, search_speed=0.3, search_tilt_step=0.5,
                 search_leg_timeout=20.0, search_timeout=90.0, track_stable=5.0):
        """파라미터가 하나라도 잘못되면 ValueError — 잘못된 설정으로 노드가 시작되지 않게 한다."""
        values = (kp_pan, kp_tilt, pan_speed_limit, tilt_speed_limit,
                  pan_deadband, tilt_deadband, timeout, search_delay, search_speed, search_tilt_step,
                  search_leg_timeout, search_timeout, track_stable)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Parameters must be finite')
        if min(kp_pan, kp_tilt) < 0 or timeout <= 0 or recovery_frames < 1:
            raise ValueError('Invalid control parameters')
        for limit in (pan_speed_limit, tilt_speed_limit, search_speed):
            if not 0 < limit <= MAX_VELOCITY_RAD_S:
                raise ValueError(f'Speed limit must be in (0, {MAX_VELOCITY_RAD_S}] rad/s')
        for deadband in (pan_deadband, tilt_deadband):
            if not 0 <= deadband < 1:
                raise ValueError('Deadband must be in [0, 1)')
        if pan_direction not in (-1, 1) or tilt_direction not in (-1, 1):
            raise ValueError('Directions must be -1 or +1')
        if min(search_delay, track_stable) < 0 or min(search_tilt_step, search_leg_timeout, search_timeout) <= 0:
            raise ValueError('Invalid search parameters')
        self.kp_pan, self.kp_tilt = kp_pan, kp_tilt
        self.pan_speed_limit, self.tilt_speed_limit = pan_speed_limit, tilt_speed_limit
        self.pan_deadband, self.tilt_deadband = pan_deadband, tilt_deadband
        self.timeout, self.recovery_frames = timeout, recovery_frames
        self.pan_direction, self.tilt_direction = pan_direction, tilt_direction
        self.search_enabled = bool(search_enabled)
        self.search_delay, self.search_tilt_step = search_delay, search_tilt_step
        # 탐색 속도는 축별 상한을 넘지 않는다
        self.search_speed = (min(search_speed, pan_speed_limit), min(search_speed, tilt_speed_limit))
        self.search_leg_timeout, self.search_timeout = search_leg_timeout, search_timeout
        self.track_stable = track_stable
        self.enabled = True
        self.state = 'IDLE'
        self.idle_reason = 'start'  # start | disabled | search_done | out_of_range
        self.count = 0              # 복귀용 연속 신선 검출 프레임 수 (타이머 호출이 아니라 새 영상 기준)
        self.last_received = None   # 마지막 신선한 입력의 수신 단조 시각 (초)
        self.last_stamp = None      # 마지막으로 받아들인 원본 영상 시각 (ns) — 중복·역순 차단
        self.frame_stamp = None
        self.xy = (0.0, 0.0)
        self.lost_at = None         # LOST로 들어간 단조 시각 — 탐색 시작 지연 기준
        self.search = None          # 탐색 패턴 진행 상태 (SEARCHING일 때만)
        self.edge_block = None      # (axis, 원시 방향): 경계에 막힌 방향 — 그쪽으로 명령하지 않음
        self.inward_since = None    # 막힌 축을 안쪽으로 명령하기 시작한 단조 시각
        self.limit_retried = False  # 이번 추적 구간에서 경계 때문에 이미 한 번 탐색했는가
        self.tracking_since = None
        self.last_sign = {'pan': 1, 'tilt': 1}  # 마지막으로 목표를 쫓던 원시 방향 — 탐색 시작 방향

    # ── 상태 전환 ───────────────────────────────────────────────────────────
    def lose(self, mono_now=None):
        """LOST로 전환: 복귀 카운트와 마지막 오차를 버린다 (이전 좌표 재사용 금지).
        탐색을 끝내고 멈춘 IDLE(search_done/out_of_range)은 그대로 둔다 (이미 정지 상태, 탐색 반복 금지)."""
        self.count = 0
        self.xy = (0.0, 0.0)
        if self.state == 'IDLE' and self.idle_reason in ('search_done', 'out_of_range'):
            return
        if self.state != 'LOST':
            self.lost_at = mono_now
        self.state = 'LOST'
        self.search = None

    def go_idle(self, reason):
        self.state, self.idle_reason = 'IDLE', reason
        self.count = 0
        self.xy = (0.0, 0.0)
        self.search = None
        if reason == 'search_done':
            self.limit_retried = False  # 범위 전체에서 못 찾았다 → 다음에 나타나면 새 구간

    def start_tracking(self, mono_now):
        self.state = 'TRACKING'
        self.tracking_since = mono_now
        self.search = None

    def start_search(self, mono_now, edge=None):
        """탐색 시작. edge=(axis, sign)이면 그 경계에 이미 닿아 있는 상태에서 시작한다."""
        self.state = 'SEARCHING'
        self.count = 0
        self.xy = (0.0, 0.0)
        self.search = dict(phase='LOCAL', started=mono_now, phase_started=mono_now, step_until=None,
                           pan_dir=self.last_sign['pan'], tilt_dir=self.last_sign['tilt'],
                           local_edges=0, tilt_edge_known=False, tilt_end=False)
        if edge:
            axis, sign = edge
            if axis == 'pan':   # 이미 그 Pan 경계에 있음 → 반대쪽으로 한 번 훑으면 LOCAL 끝
                self.search['pan_dir'] = -sign
                self.search['local_edges'] = 1
            else:               # 이미 그 Tilt 경계에 있음 → TILT_EDGE 단계는 건너뜀
                self.search['tilt_dir'] = sign
                self.search['tilt_edge_known'] = True

    def set_enabled(self, enabled):
        """명시적 추적 중지/재개(탐색 취소 포함). 중지하면 IDLE + 정지, 재개해도 3프레임 복귀 조건을 다시 거친다."""
        self.enabled = bool(enabled)
        self.go_idle('start' if self.enabled else 'disabled')
        self.edge_block = self.inward_since = None
        self.limit_retried = False

    # ── 입력 ────────────────────────────────────────────────────────────────
    def reachable(self, x, y):
        """경계에 막힌 축이 있으면, 그 축을 막힌 방향으로 움직여야 하는 목표는 범위 밖으로 본다."""
        if self.edge_block is None:
            return True
        axis, sign = self.edge_block
        if axis == 'pan':
            err, deadband, direction = x, self.pan_deadband, self.pan_direction
        else:
            err, deadband, direction = y, self.tilt_deadband, self.tilt_direction
        return abs(err) <= deadband or _sign(direction * err) != sign

    def receive(self, x, y, z, stamp_ns, ros_now_ns, mono_now):
        """/target 한 프레임 처리.

        stamp_ns   : 원본 Color 영상 시각 (header.stamp)
        ros_now_ns : 노드 ROS 시각 — bag 재생(use_sim_time)에서도 같은 시계로 비교된다
        mono_now   : 수신 단조 시각 — 시스템 시각이 바뀌어도 timeout이 늘어나지 않게 함
        """
        # timeout을 먼저 평가: 발행이 끊겼다 재개된 첫 프레임이 옛 TRACKING을 이어받지 못하게 한다.
        self.output(mono_now, ros_now_ns)
        if not self.enabled:
            return
        age = (ros_now_ns - stamp_ns) / 1e9
        valid = (all(math.isfinite(v) for v in (x, y, z))
                 and abs(x) <= 1 and abs(y) <= 1 and 0 <= z <= 1)
        # 신선도: 영상 시각이 현재보다 미래가 아니고 timeout 이내. 멈춘 카메라의 옛 영상을
        # 새 입력으로 취급하지 않는다.
        fresh = stamp_ns > 0 and 0 <= age <= self.timeout
        ordered = self.last_stamp is None or stamp_ns > self.last_stamp
        if not valid or not fresh or not ordered:
            self.lose(mono_now)
            return
        self.last_stamp = stamp_ns
        self.frame_stamp = stamp_ns
        self.last_received = mono_now
        if z == 0:  # 정상 영상의 미검출: 입력은 신선하지만 목표가 없음
            if self.state == 'SEARCHING':
                self.count = 0      # 계속 탐색
            else:
                self.lose(mono_now)  # TRACKING → 즉시 정지 (IDLE 탐색 종료 상태는 유지)
            return
        if self.state != 'TRACKING':
            if not self.reachable(x, y):  # 범위 밖에 있는 목표 — 다시 쫓아 경계를 치지 않는다
                self.count = 0
                return
            self.xy = (x, y)
            self.count += 1
            if self.count >= self.recovery_frames:
                self.start_tracking(mono_now)
            return
        self.xy = (x, y)

    def on_limit(self, axis, sign, mono_now):
        """보드 경계 정지 1건 (/opencr/limit). 보드는 이미 두 축을 멈췄고 ARM은 유지됐다."""
        if not self.enabled or axis not in AXES or sign not in (-1, 1):
            return
        if self.state == 'TRACKING':
            self.edge_block, self.inward_since = (axis, sign), None
            if not self.search_enabled:
                return  # 탐색을 안 쓰면 그 방향만 막고 추적 유지 (경계에 붙어 대기)
            if self.limit_retried:
                self.go_idle('out_of_range')  # 다시 찾은 목표가 또 범위를 넘음 → 범위 밖에 있음
            else:
                self.limit_retried = True
                self.start_search(mono_now, edge=(axis, sign))
            return
        if self.state != 'SEARCHING':
            return  # LOST/IDLE에서는 이미 정지 명령 중 — 늦게 온 이벤트
        s = self.search
        if s['phase'] == 'LOCAL' and axis == 'pan':
            self._local_edge(mono_now)
        elif s['phase'] == 'TILT_EDGE' and axis == 'tilt':
            self._rows_start(mono_now)
        elif s['phase'] == 'SWEEP' and axis == 'pan':
            self._sweep_done(mono_now)
        elif s['phase'] == 'STEP' and axis == 'tilt':
            s['tilt_end'] = True        # Tilt 끝까지 왔다 → 이 줄을 훑으면 완료
            self._next_sweep(mono_now)

    # ── 탐색 패턴 ───────────────────────────────────────────────────────────
    def _local_edge(self, mono_now):
        s = self.search
        s['local_edges'] += 1
        s['phase_started'] = mono_now
        if s['local_edges'] < 2:
            s['pan_dir'] = -s['pan_dir']   # 같은 높이에서 반대쪽 끝까지
        elif s['tilt_edge_known']:
            self._rows_start(mono_now)
        else:
            s['phase'] = 'TILT_EDGE'

    def _rows_start(self, mono_now):
        """Tilt 한쪽 끝에 있다 → 반대쪽 끝까지 줄 단위로 훑는다 (Pan은 지금 끝에서 반대로)."""
        s = self.search
        s['tilt_dir'] = -s['tilt_dir']
        self._next_sweep(mono_now)

    def _next_sweep(self, mono_now):
        s = self.search
        s['phase'], s['phase_started'] = 'SWEEP', mono_now
        s['pan_dir'] = -s['pan_dir']

    def _sweep_done(self, mono_now):
        s = self.search
        if s['tilt_end']:
            self.go_idle('search_done')
            return
        s['phase'], s['phase_started'] = 'STEP', mono_now
        s['step_until'] = mono_now + self.search_tilt_step / self.search_speed[1]

    def _search_output(self, mono_now):
        s = self.search
        if mono_now - s['started'] >= self.search_timeout:
            self.go_idle('search_done')
            return True, 0.0, 0.0
        leg_over = lambda: mono_now - s['phase_started'] >= self.search_leg_timeout
        if s['phase'] == 'LOCAL':
            if leg_over():
                self._local_edge(mono_now)
            if s['phase'] == 'LOCAL':
                return self._guard(mono_now, False, s['pan_dir'] * self.search_speed[0], 0.0)
        if s['phase'] == 'TILT_EDGE':
            if leg_over():
                self._rows_start(mono_now)
            else:
                return self._guard(mono_now, False, 0.0, s['tilt_dir'] * self.search_speed[1])
        if s['phase'] == 'SWEEP':
            if leg_over():
                self._sweep_done(mono_now)
                if self.state != 'SEARCHING':
                    return True, 0.0, 0.0
            else:
                return self._guard(mono_now, False, s['pan_dir'] * self.search_speed[0], 0.0)
        if s['phase'] == 'STEP':
            if mono_now >= s['step_until'] or leg_over():
                self._next_sweep(mono_now)
                return self._guard(mono_now, False, s['pan_dir'] * self.search_speed[0], 0.0)
            return self._guard(mono_now, False, 0.0, s['tilt_dir'] * self.search_speed[1])
        return True, 0.0, 0.0

    def _guard(self, mono_now, stop, pan, tilt):
        """경계에 막힌 방향으로는 명령하지 않는다. 그 축을 안쪽으로 EDGE_RELEASE_SEC 이상 움직이면 막힘 해제."""
        if self.edge_block is not None:
            axis, sign = self.edge_block
            value = pan if axis == 'pan' else tilt
            if _sign(value) == -sign:
                if self.inward_since is None:
                    self.inward_since = mono_now
                elif mono_now - self.inward_since >= EDGE_RELEASE_SEC:
                    self.edge_block = self.inward_since = None
            else:
                self.inward_since = None
                if _sign(value) == sign:
                    if axis == 'pan':
                        pan = 0.0
                    else:
                        tilt = 0.0
        return stop, pan, tilt

    # ── 출력 ────────────────────────────────────────────────────────────────
    def output(self, mono_now, ros_now_ns):
        """현재 내보낼 명령 (stop, pan_rad_s, tilt_rad_s). 20 Hz 타이머와 receive()가 호출한다."""
        if not self.enabled:  # 명시적 중지: IDLE 유지, 새 추적 명령 없음
            return True, 0.0, 0.0
        inputs_fresh = False
        if self.last_received is not None:
            age = (ros_now_ns - self.frame_stamp) / 1e9
            # 입력 침묵(수신 단조 시각) 또는 영상 시각 노화 중 하나라도 timeout이면 LOST
            if mono_now - self.last_received >= self.timeout or not 0 <= age <= self.timeout:
                self.lose(mono_now)
            else:
                inputs_fresh = True
        if (self.state == 'LOST' and self.search_enabled and inputs_fresh
                and self.lost_at is not None and mono_now - self.lost_at >= self.search_delay):
            self.start_search(mono_now)
        if self.state == 'SEARCHING':
            return self._search_output(mono_now)
        if self.state != 'TRACKING':
            return True, 0.0, 0.0
        if self.tracking_since is not None and mono_now - self.tracking_since >= self.track_stable:
            self.limit_retried = False  # 한동안 범위 안에서 잘 쫓았다 → 다음 경계는 새 일로 본다
        x, y = self.xy

        def clamp(value, limit):
            return max(-limit, min(limit, value))

        pan = 0.0 if abs(x) <= self.pan_deadband else clamp(
            self.pan_direction * self.kp_pan * x, self.pan_speed_limit)
        tilt = 0.0 if abs(y) <= self.tilt_deadband else clamp(
            self.tilt_direction * self.kp_tilt * y, self.tilt_speed_limit)
        for axis, value in (('pan', pan), ('tilt', tilt)):
            if value:
                self.last_sign[axis] = _sign(value)
        return self._guard(mono_now, False, pan, tilt)
