import unittest
from realsense_tracker.control_core import Controller, MAX_VELOCITY_RAD_S

class CoreTests(unittest.TestCase):
    def frames(self,c,x=0.4,y=0,z=0.1,start=1.0,count=3):
        for i in range(count):
            now=start+i*0.05;c.receive(x,y,z,int(now*1e9),int(now*1e9),now)
        return now
    def test_seven_inputs(self):
        for x,y,p,t in [(0,0,0,0),(.4,0,-.04,0),(-.4,0,.04,0),(0,.4,0,.04),(0,-.4,0,-.04)]:
            c=Controller();now=self.frames(c,x,y);stop,a,b=c.output(now,int(now*1e9))
            self.assertFalse(stop);self.assertAlmostEqual(a,p);self.assertAlmostEqual(b,t)
        c=Controller();self.frames(c);c.receive(0,0,0,1200000000,1200000000,1.2)
        self.assertEqual(c.output(1.2,1200000000),(True,0,0));self.assertEqual(c.state,'LOST')
        c=Controller();self.frames(c);self.assertEqual(c.output(1.7,1700000000),(True,0,0))
        self.assertEqual(c.state,'LOST')
    def test_direction_parameters(self):
        c=Controller(pan_direction=1,tilt_direction=-1);now=self.frames(c,.4,.4)
        stop,pan,tilt=c.output(now,int(now*1e9));self.assertFalse(stop)
        self.assertAlmostEqual(pan,.04);self.assertAlmostEqual(tilt,-.04)
        with self.assertRaises(ValueError):Controller(pan_direction=0)
        with self.assertRaises(ValueError):Controller(tilt_direction=2)
    def test_recovery_after_silence(self):
        c=Controller();self.frames(c)
        self.frames(c,start=2,count=1);self.assertEqual(c.state,'LOST')
        self.frames(c,start=2.05,count=1);self.assertEqual(c.state,'LOST')
        self.frames(c,start=2.1,count=1);self.assertEqual(c.state,'TRACKING')
    def test_stale_duplicate_invalid_and_future(self):
        for args in [(0.4,0,.1,500000000,1200000000,1.2),
                     (0.4,0,.1,1100000000,1200000000,1.2),
                     (float('nan'),0,.1,1200000000,1200000000,1.2),
                     (0.4,0,.1,1300000000,1200000000,1.2)]:
            c=Controller();self.frames(c);c.receive(*args);self.assertEqual(c.state,'LOST')
    def test_deadband_clamp_and_initial_state(self):
        c=Controller();self.assertEqual(c.output(0,0),(True,0,0));self.assertEqual(c.state,'IDLE')
        now=self.frames(c,.02,-.02);self.assertEqual(c.output(now,int(now*1e9)),(False,0,0))
        now=self.frames(c,1,-1,start=1.2);self.assertEqual(c.output(now,int(now*1e9)),(False,-.05,-.05))
    def test_source_stamp_expires_even_if_receipt_recent(self):
        c=Controller()
        for i in range(3):
            now=1.0+i*.05
            c.receive(.4,0,.1,int((now-.4)*1e9),int(now*1e9),now)
        self.assertEqual(c.state,'TRACKING')
        self.assertEqual(c.output(1.21,1210000000),(True,0,0))

    def test_per_axis_limits_and_deadbands(self):
        c=Controller(pan_speed_limit=.02,tilt_speed_limit=.05,pan_deadband=.1,tilt_deadband=0)
        now=self.frames(c,.05,1);self.assertEqual(c.output(now,int(now*1e9)),(False,0,.05))
        now=self.frames(c,-1,.05,start=1.2);stop,pan,tilt=c.output(now,int(now*1e9))
        self.assertAlmostEqual(pan,.02);self.assertAlmostEqual(tilt,.005)
    def test_speed_limit_cannot_exceed_firmware_ceiling(self):
        with self.assertRaises(ValueError):Controller(pan_speed_limit=MAX_VELOCITY_RAD_S+.01)
        with self.assertRaises(ValueError):Controller(tilt_speed_limit=0)
    def test_explicit_disable_is_idle_and_needs_three_frames(self):
        c=Controller();self.frames(c);self.assertEqual(c.state,'TRACKING')
        c.set_enabled(False);self.assertEqual(c.output(1.11,1110000000),(True,0,0))
        self.frames(c,start=1.15,count=3);self.assertEqual(c.state,'IDLE')
        self.assertEqual(c.output(2.0,2000000000),(True,0,0));self.assertEqual(c.state,'IDLE')
        c.set_enabled(True);self.frames(c,start=2.0,count=2);self.assertNotEqual(c.state,'TRACKING')
        self.frames(c,start=2.1,count=1);self.assertEqual(c.state,'TRACKING')

if __name__=='__main__':unittest.main()


class SearchTests(unittest.TestCase):
    """목표가 없을 때 탐색(SEARCHING)과 보드 경계 정지 처리. 시각은 초 단위 단조 시각 = 영상 시각."""
    def make(self, **kw):
        args = dict(kp_pan=1.0, kp_tilt=1.0, pan_speed_limit=.5, tilt_speed_limit=.5, search_enabled=True,
                    search_delay=3.0, search_speed=.3, search_tilt_step=.6, search_leg_timeout=20.0,
                    search_timeout=90.0, track_stable=5.0)
        args.update(kw)
        self.t = 1.0
        return Controller(**args)
    def feed(self, c, seconds, x=0.0, y=0.0, z=0.0):
        """20 Hz 프레임을 seconds 동안 넣고 마지막 출력을 돌려준다."""
        end = self.t + seconds
        out = None
        while self.t < end - 1e-9:
            self.t += .05
            ns = int(self.t * 1e9)
            c.receive(x, y, z, ns, ns, self.t)
            out = c.output(self.t, ns)
        return out
    def out(self, c):
        return c.output(self.t, int(self.t * 1e9))
    def tracking(self, c, x=.4, y=0.0):
        self.feed(c, .15, x, y, .05)
        self.assertEqual(c.state, 'TRACKING')

    def test_lost_waits_then_searches_only_while_frames_arrive(self):
        c = self.make(); self.tracking(c)
        self.assertEqual(self.feed(c, 2.5), (True, 0, 0)); self.assertEqual(c.state, 'LOST')   # 2 s 가림 시험 구간: 정지
        stop, pan, tilt = self.feed(c, .6)
        self.assertEqual(c.state, 'SEARCHING'); self.assertFalse(stop)
        self.assertAlmostEqual(pan, -.3); self.assertEqual(tilt, 0.0)       # 같은 높이에서 마지막으로 쫓던 Pan 방향(−)부터
        c2 = self.make(); self.tracking(c2)
        self.t += 10.0                                                       # 입력 자체가 끊김
        self.assertEqual(self.out(c2), (True, 0, 0)); self.assertEqual(c2.state, 'LOST')
    def test_short_occlusion_recovers_in_view_without_search(self):
        c = self.make(); self.tracking(c)
        self.feed(c, 2.0); self.assertEqual(c.state, 'LOST')
        self.feed(c, .15, .1, 0, .05); self.assertEqual(c.state, 'TRACKING')
    def test_pattern_turns_on_board_limits_and_ends_idle(self):
        c = self.make(); self.tracking(c); self.feed(c, 3.1)
        self.assertEqual(c.state, 'SEARCHING'); self.assertEqual(self.feed(c, .1)[1:], (-.3, 0.0))   # LOCAL: 같은 높이
        c.on_limit('pan', -1, self.t); self.assertEqual(self.feed(c, .1)[1:], (.3, 0.0))    # LOCAL: 반대쪽 끝까지
        c.on_limit('pan', 1, self.t); self.assertEqual(self.feed(c, .1)[1:], (0.0, .3))     # TILT_EDGE: Tilt 기본 + 끝으로
        c.on_limit('tilt', 1, self.t); self.assertEqual(self.feed(c, .1)[1:], (-.3, 0.0))   # 첫 줄: Pan 반대로
        c.on_limit('pan', -1, self.t)                                                        # 한 줄 끝 → STEP
        self.assertEqual(self.feed(c, 1.0)[1:], (0.0, -.3))
        self.assertEqual(self.feed(c, 1.1)[1:], (.3, 0.0))                                   # 0.6/0.3=2 s 후 다음 줄
        c.on_limit('pan', 1, self.t); self.assertEqual(self.feed(c, .5)[1:], (0.0, -.3))
        c.on_limit('tilt', -1, self.t)                                                       # Tilt 반대쪽 끝 → 마지막 줄
        self.assertEqual(self.feed(c, .1)[1:], (-.3, 0.0))
        c.on_limit('pan', -1, self.t)
        self.assertEqual(self.feed(c, .1), (True, 0, 0)); self.assertEqual((c.state, c.idle_reason), ('IDLE', 'search_done'))
        self.feed(c, 10.0); self.assertEqual(c.state, 'IDLE')                                # 다시 탐색하지 않음
        self.feed(c, .15, -.2, 0, .05); self.assertEqual(c.state, 'TRACKING')               # 나타나면 추적
    def test_search_finds_target(self):
        c = self.make(); self.tracking(c); self.feed(c, 3.5); self.assertEqual(c.state, 'SEARCHING')
        self.feed(c, .1, .6, .2, .05); self.assertEqual(c.state, 'SEARCHING')
        stop, pan, tilt = self.feed(c, .05, .6, .2, .05)
        self.assertEqual(c.state, 'TRACKING'); self.assertFalse(stop)
        self.assertAlmostEqual(pan, -.5); self.assertAlmostEqual(tilt, .2)                  # 0.5 rad/s 상한
    def test_search_without_board_events_times_out(self):
        c = self.make(search_leg_timeout=2.0, search_timeout=12.0); self.tracking(c); self.feed(c, 3.1)
        self.assertEqual(c.state, 'SEARCHING')
        self.feed(c, 12.0); self.assertEqual((c.state, c.idle_reason), ('IDLE', 'search_done'))
        self.assertEqual(self.out(c), (True, 0, 0))
    def test_timeout_during_search_stops(self):
        c = self.make(); self.tracking(c); self.feed(c, 3.1); self.assertEqual(c.state, 'SEARCHING')
        self.t += .6; self.assertEqual(self.out(c), (True, 0, 0)); self.assertEqual(c.state, 'LOST')
    def test_limit_while_tracking_searches_then_out_of_range_idle(self):
        c = self.make(); self.tracking(c, x=-.8)                             # Pan 원시 + 방향으로 추적
        self.assertAlmostEqual(self.out(c)[1], .5)
        c.on_limit('pan', 1, self.t); self.assertEqual(c.state, 'SEARCHING'); self.assertEqual(c.edge_block, ('pan', 1))
        self.feed(c, .05, -.8, 0, .05); self.assertEqual(c.count, 0)       # 막힌 방향의 목표는 세지 않음
        self.assertAlmostEqual(self.feed(c, .3)[1], -.3); self.assertIsNotNone(c.edge_block)   # 막 출발: 아직 경계 근처
        self.feed(c, .3); self.assertIsNone(c.edge_block)                  # 안쪽으로 0.5 s 이상 → 막힘 해제
        self.feed(c, .15, -.5, 0, .05); self.assertEqual(c.state, 'TRACKING')   # 다시 보임 → 추적
        c.on_limit('pan', 1, self.t)                                         # track_stable 안에 또 범위 초과
        self.assertEqual((c.state, c.idle_reason), ('IDLE', 'out_of_range')); self.assertEqual(self.out(c), (True, 0, 0))
        self.feed(c, 1.0, -.5, 0, .05); self.assertEqual(c.state, 'IDLE')   # 여전히 범위 밖 → 대기
        self.feed(c, .15, .3, 0, .05); self.assertEqual(c.state, 'TRACKING')    # 안쪽으로 돌아옴 → 추적
        self.assertLess(self.out(c)[1], 0)
        self.feed(c, .6, .3, 0, .05); self.assertIsNone(c.edge_block)
    def test_tilt_limit_search_skips_tilt_edge_phase(self):
        c = self.make(); self.tracking(c, x=0, y=.8)                         # Tilt 원시 + 로 추적 (dir +1 기본값 아님 → kp·ey 부호)
        sign = 1 if c.tilt_direction * .8 > 0 else -1
        c.on_limit('tilt', sign, self.t); self.assertEqual(c.state, 'SEARCHING')
        self.assertEqual(self.feed(c, .1)[2], 0.0)                           # LOCAL: Pan만
        c.on_limit('pan', 1, self.t); c.on_limit('pan', -1, self.t)
        stop, pan, tilt = self.feed(c, .1)
        self.assertEqual(c.search['phase'], 'SWEEP'); self.assertEqual(tilt, 0.0)   # 이미 Tilt 끝이라 바로 줄 훑기
        self.assertEqual(c.search['tilt_dir'], -sign)
    def test_stable_tracking_resets_limit_retry(self):
        c = self.make(); self.tracking(c, x=-.8); c.on_limit('pan', 1, self.t)
        self.feed(c, .7); self.feed(c, .15, -.5, 0, .05); self.assertEqual(c.state, 'TRACKING')
        self.feed(c, 5.2, .1, 0, .05); self.assertFalse(c.limit_retried)
        self.feed(c, .1, -.8, 0, .05); c.on_limit('pan', 1, self.t); self.assertEqual(c.state, 'SEARCHING')
    def test_limit_without_search_blocks_only_outward(self):
        c = self.make(search_enabled=False); self.tracking(c, x=-.8, y=.4)
        c.on_limit('pan', 1, self.t); self.assertEqual(c.state, 'TRACKING')
        stop, pan, tilt = self.out(c); self.assertEqual(pan, 0.0); self.assertAlmostEqual(tilt, .4)
        self.feed(c, .05, .4, 0, .05); self.assertAlmostEqual(self.out(c)[1], -.4)   # 안쪽 명령은 허용
        self.feed(c, .6, .4, 0, .05); self.assertIsNone(c.edge_block)
        self.feed(c, 5.0); self.assertEqual(c.state, 'LOST')                 # 탐색 꺼짐: 정지 유지
    def test_disable_cancels_search_and_ignores_late_limits(self):
        c = self.make(); self.tracking(c); self.feed(c, 3.1); self.assertEqual(c.state, 'SEARCHING')
        c.set_enabled(False); self.assertEqual(self.out(c), (True, 0, 0)); self.assertEqual(c.state, 'IDLE')
        c.on_limit('pan', 1, self.t); self.assertEqual(c.state, 'IDLE')
    def test_search_speed_respects_axis_limits_and_validation(self):
        c = self.make(search_speed=.5, pan_speed_limit=.2); self.tracking(c); self.feed(c, 3.1)
        self.assertAlmostEqual(abs(self.out(c)[1]), .2)
        c.on_limit('pan', -1, self.t); c.on_limit('pan', 1, self.t); self.assertAlmostEqual(abs(self.out(c)[2]), .5)
        for bad in (dict(search_speed=0), dict(search_speed=.6), dict(search_tilt_step=0), dict(search_timeout=0),
                    dict(search_delay=-1)):
            with self.assertRaises(ValueError): self.make(**bad)
