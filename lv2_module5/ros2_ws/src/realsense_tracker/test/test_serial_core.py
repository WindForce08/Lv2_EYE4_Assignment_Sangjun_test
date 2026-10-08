import unittest
from realsense_tracker.serial_core import SerialBridge,check_mode
from realsense_tracker.control_core import MAX_VELOCITY_RAD_S


def state(name='BOOT',armed=0,goal='0,0',vel='0,0',torque='0,0',mode='DRY',age=0,t=10):
    return f'STATE {name} MODE={mode} ARMED={armed} GOAL={goal} POS=3078,4096 VEL={vel} TORQUE={torque} AGE_MS={age} T_MS={t}'


class Fake:
    def __init__(self):self.rx=b'';self.tx=[];self.closed=False
    def read(self):data=self.rx;self.rx=b'';return data
    def write(self,data):self.tx.append(data.decode().strip())
    def close(self):self.closed=True


class SerialTests(unittest.TestCase):
    def setUp(self,mode='DRY'):
        self.now=100.;self.io=Fake();self.logs=[];self.mode=mode
        self.bridge=SerialBridge(self.io,lambda *r:self.logs.append(r),lambda:self.now,expected_mode=mode)
    def line(self,text):self.io.rx+=(text+'\n').encode();self.bridge.tick()
    def prepared(self):
        self.line(state(mode=self.mode));self.assertEqual(self.io.tx,['STATUS'])
        self.assertTrue(self.bridge.prepare()[0]);self.line('CHECK_OK BOTH_TORQUES_OFF')
        self.line('ACK HOLD ZERO_REQUESTED');self.line('EVENT STOPPED TORQUE_RETAINED')
        self.line(state('DISARMED',torque='1,1',mode=self.mode));self.assertEqual(self.bridge.phase,'READY')
    def command(self,p=.048,t=0,stop=False,stamp=1_000_000_000,now=1_000_000_000):
        return self.bridge.receive_command(stop,p,t,stamp,now)
    def poll(self):
        if not self.bridge.pending:self.bridge.send('STATUS','STATUS')
    def armed(self):
        self.prepared();self.command();self.assertTrue(self.bridge.arm()[0])
        self.line('ACK ARM');self.line(state('ARMED',1,torque='1,1',mode=self.mode))
        self.assertEqual(self.bridge.phase,'ARMED')
    def test_explicit_prepare_arm_and_no_sign_flip(self):
        self.line(state(mode=self.mode));self.command(-.04,.04)
        self.bridge.tick();self.assertNotIn('ARM',self.io.tx)
        self.prepared_from_boot()
        self.command(-.04,.04,stamp=2_000_000_000,now=2_000_000_000)
        self.assertTrue(self.bridge.arm()[0]);self.line('ACK ARM')
        self.line(state('ARMED',1,torque='1,1',mode=self.mode))
        self.assertIn('VEL -0.040000 0.040000',self.io.tx)
    def prepared_from_boot(self):
        self.assertTrue(self.bridge.prepare()[0]);self.line('CHECK_OK BOTH_TORQUES_OFF')
        self.line('ACK HOLD ZERO_REQUESTED');self.line('EVENT STOPPED TORQUE_RETAINED')
        self.line(state('DISARMED',torque='1,1',mode=self.mode))
    def test_live_mode_refused_before_prepare(self):
        self.line(state(mode='LIVE'));self.assertEqual(self.bridge.phase,'FAULT')
        self.assertEqual(self.io.tx,['STATUS']);self.assertFalse(self.bridge.prepare()[0])
    def test_non_boot_session_not_adopted(self):
        self.line(state('DISARMED',torque='1,1',mode=self.mode));self.assertEqual(self.bridge.phase,'FAULT')
        self.assertNotIn('ARM',self.io.tx)
    def test_fragmented_state_and_malformed_response(self):
        raw=(state()+'\n').encode();self.io.rx=raw[:20];self.bridge.tick()
        self.assertEqual(self.bridge.phase,'CONNECTING')
        self.io.rx=raw[20:];self.bridge.tick();self.assertEqual(self.bridge.phase,'BOOT')
        self.line('STATE broken');self.assertEqual(self.bridge.reason,'MALFORMED_STATE')
    def test_invalid_commands_latch_when_armed(self):
        for args in [(float('nan'),0,2_000_000_000,2_000_000_000),
                     (MAX_VELOCITY_RAD_S+.01,0,2_000_000_000,2_000_000_000),
                     (.04,0,1_000_000_000,1_000_000_000),
                     (.04,0,2_000_000_000,2_200_000_000),
                     (.04,0,2_000_000_000,1_900_000_000)]:
            self.setUp();self.armed();p,t,stamp,now=args
            self.assertFalse(self.command(p,t,stamp=stamp,now=now))
            self.assertEqual(self.bridge.phase,'FAULT');self.assertFalse(self.bridge.arm()[0])
    def test_missing_ack_is_not_retried(self):
        self.line(state(mode=self.mode));self.bridge.prepare();self.now+=2.1;self.bridge.tick()
        self.assertEqual(self.bridge.phase,'FAULT');self.assertEqual(self.io.tx.count('CHECK'),1)
    def test_slow_live_check_reaches_ready(self):
        # Measured LIVE timing: CHECK blocks ~1.25 s (no STATE), HOLD ack ~0.15 s, STOPPED ~0.09 s later.
        self.setUp('LIVE');self.line(state(mode='LIVE'));self.bridge.prepare()
        for _ in range(125):self.now+=.01;self.bridge.tick()
        self.assertEqual(self.bridge.phase,'CHECKING')
        self.line('CHECK_OK BOTH_TORQUES_OFF');self.assertEqual(self.bridge.phase,'HOLDING')
        for _ in range(15):self.now+=.01;self.bridge.tick()
        self.line('ACK HOLD ZERO_REQUESTED')
        for _ in range(9):self.now+=.01;self.bridge.tick()
        self.line('EVENT STOPPED TORQUE_RETAINED')
        self.line(state('DISARMED',torque='1,1',mode='LIVE',t=2000))
        self.assertEqual(self.bridge.phase,'READY');self.assertEqual(self.bridge.reason,'')
    def test_missing_arm_ack_no_vel_or_retry(self):
        self.prepared();self.command();self.bridge.arm();self.now+=.21
        self.command(stamp=2_000_000_000,now=2_000_000_000);self.bridge.tick()
        self.assertEqual(self.bridge.phase,'FAULT');self.assertEqual(self.io.tx.count('ARM'),1)
        self.assertEqual(self.bridge.reason,'RESPONSE_TIMEOUT: ARM')
        self.assertFalse(any(x.startswith('VEL') for x in self.io.tx))
    def test_command_silence_disarms_without_rearm(self):
        self.armed();self.now+=.16;self.bridge.tick()
        self.assertEqual(self.bridge.reason,'ROS_COMMAND_TIMEOUT');self.assertEqual(self.io.tx[-1],'DISARM')
        self.command(stamp=2_000_000_000,now=2_000_000_000);self.bridge.tick()
        self.assertEqual(self.io.tx.count('ARM'),1)
    def test_board_timeout_and_fault_latch(self):
        for line in ['EVENT TIMEOUT DISARMED ZERO_REQUESTED','FAULT READ SUPPORT_CAMERA; NO_AUTO_RECOVERY']:
            self.setUp();self.armed();self.line(line)
            self.assertEqual(self.bridge.phase,'FAULT');self.assertFalse(self.bridge.arm()[0])
    def test_status_age_and_reboot(self):
        self.prepared();self.line(state('DISARMED',torque='1,1',age=150))
        self.assertEqual(self.bridge.reason,'STALE_BOARD_FEEDBACK')
        self.setUp();self.prepared();self.line(state('DISARMED',torque='1,1',t=1))
        self.assertEqual(self.bridge.reason,'BOARD_REBOOT')
    def test_stop_is_stop_not_torque_release(self):
        self.armed();self.line('ACK VEL')
        self.command(stop=True,stamp=2_000_000_000,now=2_000_000_000);self.bridge.tick()
        self.assertEqual(self.io.tx[-1],'STOP');self.assertNotIn('SUPPORTED_OFF',self.io.tx)
    def test_disconnect_latches_without_reconnect(self):
        self.prepared()
        def broken():raise OSError('peer disconnected')
        self.io.read=broken;self.bridge.tick()
        self.assertEqual(self.bridge.phase,'FAULT')
        self.assertIn('TRANSPORT_READ',self.bridge.reason)
        self.assertFalse(self.bridge.prepare()[0])
    def test_old_stopped_event_cannot_authorize_vel_after_new_stop(self):
        self.armed();self.line('ACK VEL')
        self.command(stop=True,stamp=2_000_000_000,now=2_000_000_000)
        self.bridge.tick();self.assertEqual(self.io.tx[-1],'STOP')
        # Event from an older completed stop arrives before the new STOP ACK.
        self.line('EVENT STOPPED TORQUE_RETAINED');self.line('ACK STOP ZERO_REQUESTED')
        self.command(-.04,0,stamp=3_000_000_000,now=3_000_000_000)
        self.bridge.tick()
        self.assertEqual(self.io.tx[-1],'STOP')
        self.assertTrue(self.bridge.stopping)
        self.assertEqual(self.io.tx.count('STOP'),1)
    def test_slow_stop_keeps_board_timer_alive_then_resumes(self):
        # Field 2026-10-07: a stop that rang for ~0.31 s let the board's 300 ms command timeout fire.
        self.armed();self.line('ACK VEL')
        stamp=2_000_000_000
        self.command(stop=True,stamp=stamp,now=stamp);self.bridge.tick()
        self.assertEqual(self.io.tx[-1],'STOP');self.line('ACK STOP ZERO_REQUESTED')
        for _ in range(4):   # 0.2 s (> STOP_KEEPALIVE) of fresh 20 Hz stop commands while the board still reports STOPPING
            self.now+=.05;stamp+=50_000_000
            self.command(stop=True,stamp=stamp,now=stamp);self.bridge.tick()
            self.line(state('STOPPING',1,torque='1,1',mode=self.mode))
        self.assertEqual(self.io.tx.count('STOP'),2);self.assertEqual(self.io.tx[-1],'STOP')
        self.line('ACK STOP ZERO_REQUESTED');self.assertEqual(self.bridge.phase,'ARMED')
        self.now+=.05;stamp+=50_000_000
        self.command(-.04,0,stamp=stamp,now=stamp);self.bridge.tick()
        self.line(state('ARMED',1,torque='1,1',mode=self.mode))
        self.assertFalse(self.bridge.stopping);self.assertEqual(self.io.tx[-1],'VEL -0.040000 0.000000')
        self.assertEqual(self.io.tx.count('STOP'),2)
    def test_status_confirmation_required_before_resuming_vel(self):
        self.armed();self.line('ACK VEL')
        self.command(stop=True,stamp=2_000_000_000,now=2_000_000_000)
        self.bridge.tick();self.line('ACK STOP ZERO_REQUESTED')
        self.line('EVENT STOPPED TORQUE_RETAINED')
        self.assertTrue(self.bridge.stopping)
        # Only the reply to a newer STATUS confirms the current board state.
        self.bridge.send('STATUS','STATUS')
        self.line(state('STOPPING',1,torque='1,1'));self.assertTrue(self.bridge.stopping)
        self.bridge.send('STATUS','STATUS')
        self.line(state('ARMED',1,torque='1,1',mode=self.mode));self.assertFalse(self.bridge.stopping)
        self.command(-.04,0,stamp=3_000_000_000,now=3_000_000_000)
        self.bridge.tick();self.assertEqual(self.io.tx[-1],'VEL -0.040000 0.000000')
    def test_limit_stop_answers_vel_keeps_arm_and_resumes_after_status(self):
        self.armed();self.assertEqual(self.bridge.pending[0],'VEL')   # VEL sent once ARMED was confirmed
        self.line('EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN DIR=+1')
        self.assertEqual(self.bridge.phase,'ARMED');self.assertIsNone(self.bridge.pending)
        self.assertTrue(self.bridge.stopping);self.assertEqual(self.bridge.limit_events,[('pan',1)])
        self.assertEqual(self.bridge.snapshot()['last_limit'],('pan',1))
        self.assertNotIn('DISARM',self.io.tx)
        # Inward command while the board is still stopping: no VEL until a newer STATUS confirms the stop.
        self.command(-.04,0,stamp=2_000_000_000,now=2_000_000_000);self.bridge.tick()
        self.assertFalse(self.io.tx[-1].startswith('VEL -'))
        self.poll()
        self.line(state('STOPPING',1,torque='1,1',mode=self.mode));self.assertTrue(self.bridge.stopping)
        self.line('EVENT STOPPED TORQUE_RETAINED')
        self.poll()
        self.line(state('ARMED',1,torque='1,1',mode=self.mode,t=20));self.assertFalse(self.bridge.stopping)
        self.command(-.04,0,stamp=3_000_000_000,now=3_000_000_000);self.bridge.tick()
        self.assertEqual(self.io.tx[-1],'VEL -0.040000 0.000000');self.assertEqual(self.bridge.phase,'ARMED')
    def test_limit_stop_crossing_vel_err_stopping_ignored_once(self):
        self.armed()
        self.line('EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=TILT DIR=-1')
        self.line('ERR STOPPING');self.assertEqual(self.bridge.phase,'ARMED')   # reply to the VEL that crossed the edge
        self.assertEqual(self.bridge.limit_events,[('tilt',-1)])
        self.line('ERR STOPPING');self.assertEqual(self.bridge.phase,'FAULT')   # a second one is a real error
    def test_err_stopping_allowance_expires_with_next_status(self):
        self.armed();self.line('EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN DIR=-1')
        self.poll()
        self.line(state('STOPPING',1,torque='1,1',mode=self.mode,t=20))
        self.line('ERR STOPPING');self.assertEqual(self.bridge.phase,'FAULT')
    def test_limit_events_that_are_still_faults(self):
        for line in ['EVENT LIMIT DISARMED ZERO_REQUESTED','EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=YAW DIR=+1']:
            self.setUp();self.armed();self.line(line)
            self.assertEqual(self.bridge.phase,'FAULT');self.assertEqual(self.io.tx[-1],'DISARM')
        self.setUp();self.prepared();self.line('EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN DIR=+1')
        self.assertEqual(self.bridge.phase,'FAULT')   # only legitimate while ARMED
    def test_close_disarms_and_closes(self):
        self.prepared();self.bridge.close();self.assertTrue(self.io.closed)
        self.assertEqual(self.io.tx[-1],'DISARM')

    def test_live_board_accepted_only_when_live_expected(self):
        self.setUp('LIVE');self.armed()
        self.command(-.04,.04,stamp=2_000_000_000,now=2_000_000_000);self.line('ACK VEL')
        self.bridge.tick();self.assertIn('VEL -0.040000 0.040000',self.io.tx)
        self.assertEqual(self.bridge.snapshot()['expected_mode'],'LIVE')
        self.setUp('LIVE');self.line(state(mode='DRY'))
        self.assertEqual(self.bridge.phase,'FAULT');self.assertIn('MODE_MISMATCH',self.bridge.reason)
        self.assertEqual(self.io.tx,['STATUS'])
    def test_live_keeps_command_timeout_and_no_rearm(self):
        self.setUp('LIVE');self.armed();self.now+=.16;self.bridge.tick()
        self.assertEqual(self.bridge.reason,'ROS_COMMAND_TIMEOUT');self.assertEqual(self.io.tx[-1],'DISARM')
        self.assertFalse(self.bridge.arm()[0]);self.assertEqual(self.io.tx.count('ARM'),1)
    def test_mode_configuration_requires_explicit_live_confirmation(self):
        self.assertEqual(check_mode(True,'DRY',False),'DRY')
        self.assertEqual(check_mode(False,'LIVE',True),'LIVE')
        for args in [(False,'LIVE',False),(True,'LIVE',True),(False,'DRY',False),
                     (True,'DRY',True),(True,'live',True),(None,'LIVE',True)]:
            with self.assertRaises(RuntimeError):check_mode(*args)
        with self.assertRaises(ValueError):SerialBridge(Fake(),expected_mode='AUTO')
    def test_command_age_is_configurable_but_below_board_timeout(self):
        bridge=SerialBridge(Fake(),clock=lambda:self.now,command_age=.25)
        self.assertTrue(bridge.receive_command(False,.01,0,1_000_000_000,1_200_000_000))
        with self.assertRaises(ValueError):SerialBridge(Fake(),command_age=.3)

if __name__=='__main__':unittest.main()
