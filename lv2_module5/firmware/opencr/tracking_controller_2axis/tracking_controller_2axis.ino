// tracking_controller_2axis — OpenCR Pan/Tilt 최종 런타임 펌웨어 (담당: 제어)
//
// 입력: Raspberry Pi opencr_node(serial_core.py)의 USB 시리얼 ASCII 줄 명령, 115200 8N1, LF 종료, 최대 63자
//   STATUS | CHECK | HOLD | ARM | VEL <pan_rad_s> <tilt_rad_s> | STOP | DISARM | SUPPORTED_OFF
// 출력: STATE/ACK/ERR/EVENT/FAULT 줄 + 두 XM430-W350 (Pan ID 11, Tilt ID 12, 1 Mbps, Protocol 2.0, Velocity Mode)
//
// 이 계층의 책임 (Pi의 control_node는 P제어·방향·속도 상한·deadband·상태·target timeout 담당)
//   - 단위 변환: rad/s → Goal_Velocity 원시 단위(0.229 rpm). 부호는 이미 모터 원시 부호 (방향 재적용 없음)
//   - 속도 상한: |VEL| > MAX_RAD_S(0.5) → ERR RANGE, 두 값 중 하나라도 잘못되면 둘 다 거부
//               (Pi control_core.MAX_VELOCITY_RAD_S와 같은 값)
//   - 속도 감시: 실제 속도가 최근 목표 속도 + SPEED_MARGIN(원시 단위)보다 빠르면 FAULT (명령 대비, 고정 절대값 아님)
//   - 각도 경계: 실제 엔코더 위치 기준. 원점 ±STOP_COUNTS에서 바깥 방향 명령 → 두 축 정지, ARM 유지
//               "EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN|TILT DIR=+1|-1" (DIR = 막힌 원시 방향).
//               host(serial_core)는 이를 정상 정지로 처리하고 정지 확인 후 안쪽 명령을 재개한다 (Pan/Tilt 탐색의 반환점).
//               ±OUTER_COUNTS 초과 → FAULT (실제 기구 범위 확정 TODO — config/hardware.yaml)
//   - 명령 timeout: ARM 중 마지막 유효 명령 후 COMMAND_MS(300 ms) → 두 축 0 + DISARM (토크 유지)
//               → control_node·opencr_node·USB 어느 쪽이 멈춰도 마지막 속도로 계속 돌지 않는다
//   - 모터 Bus_Watchdog: BUS_TICKS(10 × 20 ms = 200 ms) 동안 버스 패킷이 없으면 모터 스스로 정지 (보드 멈춤 대비)
//   - 고장 처리: 읽기·쓰기 실패, 하드웨어 오류, 위치 급변, 예상 밖 속도, 정지 미확인 → FAULT 래치,
//               두 축 0 쓰기 1회 후 버스 통신 중단(watchdog 만료 유도). 자동 복구 없음 — reset + CHECK 필요
//
// DRY / LIVE (컴파일 플래그, 기본 DRY):
//   ENABLE_MOTOR_OUTPUT=0 (기본) → MODE=DRY: 모터 버스를 전혀 호출하지 않음, 피드백은 모의 값
//   ENABLE_MOTOR_OUTPUT=1        → MODE=LIVE: 실제 구동. arduino-cli --build-property로만 켠다 (README)
// 정상 정지(STOP/DISARM/timeout)는 영속도 + 토크 유지다 (Tilt 낙하 방지). 토크 해제는 SUPPORTED_OFF만.
// 아래 상수의 실측 근거와 사본: lv2_module5/config/hardware.yaml (펌웨어 상수를 바꾸면 함께 수정)
#include <Arduino.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <DynamixelWorkbench.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
extern "C" {
#include "usbd_cdc_interface.h"
}
#ifndef ENABLE_MOTOR_OUTPUT
#define ENABLE_MOTOR_OUTPUT 0
#endif
#if ENABLE_MOTOR_OUTPUT != 0 && ENABLE_MOTOR_OUTPUT != 1
#error ENABLE_MOTOR_OUTPUT_must_be_0_or_1
#endif

DynamixelWorkbench dxl;
const uint8_t IDS[2] = {11, 12};  // {Pan, Tilt} — dxl_discovery 스캔으로 확인
const uint32_t COMMAND_MS = 300, POLL_MS = 20, BUS_TICKS = 10;  // 명령 timeout, 피드백 주기, 모터 watchdog(×20 ms)
const float MAX_RAD_S = 0.5f;
const float RAD_S_PER_UNIT = 0.229f * 6.28318530718f / 60.0f;
// Velocity input is already motor-native sign (positive = encoder increasing). Which way that turns the camera
// depends on assembly; the host's control.yaml *_direction is the only place the image-to-motor sign is chosen.
// With ACCEPT_STATIONARY_POSE_AS_ORIGIN the stationary pose at CHECK is the origin; PAN/TILT_NEUTRAL are used otherwise.
const int32_t PAN_NEUTRAL = 1043;
const int32_t TILT_NEUTRAL = 2161;
const int32_t NEUTRAL_TOL_COUNTS = 100;
// OUTER = requested Pan ±120° / Tilt ±60° (rounded inward). STOP is STOP_MARGIN_COUNTS inside it: at 0.5 rad/s the
// axis can travel ≈7 counts between 20 ms polls plus ≈22 counts while decelerating (PROFILE_ACCEL 10) after the stop.
const int32_t OUTER_COUNTS[2] = {1365, 682};
const int32_t STOP_MARGIN_COUNTS = 60;
const int32_t STOP_COUNTS[2] = {OUTER_COUNTS[0] - STOP_MARGIN_COUNTS, OUTER_COUNTS[1] - STOP_MARGIN_COUNTS};  // 1305, 622
// Profile_Acceleration unit 214.577 rev/min² ≈ 0.3745 rad/s². 10 → ≈3.7 rad/s²: 0.5 rad/s stops in ≈0.13 s (≈22 counts).
// Value 1 needed ≈1.3 s, beyond the 500 ms stop check and the stop margin.
const int32_t PROFILE_ACCEL = 10;
const int32_t SPEED_MARGIN = 3;              // raw units (≈0.07 rad/s) of feedback noise/overshoot over the recent goal
const int32_t HOLD_DRIFT_COUNTS = 50;
const int32_t ARM_POSE_TOLERANCE_COUNTS = 120;
#ifndef ACCEPT_STATIONARY_POSE_AS_ORIGIN
#define ACCEPT_STATIONARY_POSE_AS_ORIGIN 1
#endif

bool ready = false, holding = false, armed = false, faulted = false;
bool stopping = false, baseline = false;
bool known[2] = {false, false};
int32_t pos[2] = {0, 0}, vel[2] = {0, 0}, torque[2] = {0, 0};
int32_t origin[2] = {PAN_NEUTRAL, TILT_NEUTRAL},
        previous[2] = {0, 0},
        goal[2] = {0, 0};
int32_t holdAnchor[2] = {0, 0};
int32_t speedCap[2] = {0, 0};   // largest |goal| the motor may still be moving at (tightens once it settles)
uint32_t lastCommand = 0, lastPoll = 0, sampleAt = 0, stopAt = 0;
uint8_t quiet = 0;
char line[64];
size_t used = 0;
bool discardLine = false;

void service();
void stopBoth(bool disarm, const char *event);

void reply(const char *s) {
  uint8_t packet[256];
  size_t n = strlen(s);
  if (n > sizeof(packet)-2) return;
  memcpy(packet, s, n);
  packet[n++] = '\n';
  (void)CDC_Itf_Write(packet, (uint32_t)n);
}
bool readItem(uint8_t axis, const char *key, int32_t &v) {
  const char *log = nullptr;
  return dxl.itemRead(IDS[axis], key, &v, &log);
}
bool eq(uint8_t axis, const char *key, int32_t v) {
  int32_t actual=0;
  return readItem(axis,key,actual) && actual==v;
}
bool writeItem(uint8_t axis, const char *key, int32_t v) {
  const char *log=nullptr;
  return dxl.itemWrite(IDS[axis],key,v,&log);
}
bool verified(uint8_t axis, const char *key, int32_t v) {
  return writeItem(axis,key,v) && eq(axis,key,v);
}
// Try BOTH zero writes even if the first fails. No other automatic bus traffic
// after fault: this permits each configured motor watchdog to expire.
void fail(const char *why) {
  if (faulted) return;
  faulted=true; armed=false; stopping=false;
  goal[0]=goal[1]=0;
#if ENABLE_MOTOR_OUTPUT
  for (uint8_t i=0;i<2;++i) if (known[i]) (void)writeItem(i,"Goal_Velocity",0);
#endif
  char out[160]; snprintf(out,sizeof(out),"FAULT %s SUPPORT_CAMERA; NO_AUTO_RECOVERY",why);
  reply(out);
}
bool goals(int32_t p, int32_t t) {
#if ENABLE_MOTOR_OUTPUT
  const bool a=verified(0,"Goal_Velocity",p);
  // If first write failed, do not initiate a nonzero second-axis movement.
  if (!a) { fail("PAN_WRITE"); return false; }
  if (!verified(1,"Goal_Velocity",t)) { fail("TILT_WRITE"); return false; }
#endif
  const int32_t next[2]={p,t};
  for(uint8_t i=0;i<2;++i) {
    const int32_t m=next[i]<0?-next[i]:next[i];
    if(m>speedCap[i])speedCap[i]=m;
    goal[i]=next[i];
  }
  return true;
}
// Stop both axes at an edge; ARM is kept so the host can continue inward after the stop completes.
void limitStop(uint8_t axis, int32_t outwardSign) {
  char out[96];
  snprintf(out,sizeof(out),"EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=%s DIR=%s",
           axis==0?"PAN":"TILT", outwardSign>0?"+1":"-1");
  stopBoth(false,out);
}
int32_t neutralOffset(int32_t p) {
  int32_t x=p%4096; if(x<0)x+=4096; if(x>=2048)x-=4096; return x;
}
int32_t relativePosition(uint8_t axis, int32_t value) {
  int32_t d=value-origin[axis];
  if (d > 2048) d-=4096;
  if (d < -2048) d+=4096;
  return d;
}
bool feedback() {
#if ENABLE_MOTOR_OUTPUT
  for(uint8_t i=0;i<2;++i) {
    int32_t error=0;
    if(!readItem(i,"Present_Position",pos[i]) ||
       !readItem(i,"Present_Velocity",vel[i]) ||
       !readItem(i,"Torque_Enable",torque[i]) ||
       !readItem(i,"Hardware_Error_Status",error)) { fail("READ"); return false; }
    if(error) { fail("HARDWARE_ERROR"); return false; }
    if(holding && (torque[i]!=1 || !eq(i,"Bus_Watchdog",BUS_TICKS))) {
      fail("TORQUE_OR_BUS_WATCHDOG"); return false;
    }
    if(baseline) {
      const int64_t d=relativePosition(i,pos[i]);
      const int64_t step=(int64_t)pos[i]-previous[i];
      if(d<=-OUTER_COUNTS[i] || d>=OUTER_COUNTS[i]) { fail("OUTER_BOUND"); return false; }
      if(step>30 || step< -30) { fail("POSITION_DISCONTINUITY"); return false; }
      if(vel[i]>speedCap[i]+SPEED_MARGIN || vel[i]< -(speedCap[i]+SPEED_MARGIN)) {
        fail("UNEXPECTED_SPEED"); return false;
      }
      // Settled near the current goal: tighten the cap from the previous (faster) goal to this one.
      if(vel[i]-goal[i]<=2 && goal[i]-vel[i]<=2) speedCap[i]=goal[i]<0?-goal[i]:goal[i];
      if(holding && !stopping && goal[i]==0) {
        const int64_t h=(int64_t)pos[i]-holdAnchor[i];
        if(h>HOLD_DRIFT_COUNTS || h< -HOLD_DRIFT_COUNTS) { fail("HOLD_DRIFT"); return false; }
      }
    }
    previous[i]=pos[i];
  }
#else
  torque[0]=torque[1]=holding?1:0; // Simulation only, clearly labelled MODE=DRY.
  vel[0]=goal[0]; vel[1]=goal[1];
#endif
  sampleAt=millis(); return true;
}
void status() {
  char out[250];
  const char *state=faulted?"FAULT":!ready?"BOOT":!holding?"READY":
                    stopping?"STOPPING":armed?"ARMED":"DISARMED";
  snprintf(out,sizeof(out),
    "STATE %s MODE=%s ARMED=%d GOAL=%ld,%ld POS=%ld,%ld VEL=%ld,%ld TORQUE=%ld,%ld AGE_MS=%lu T_MS=%lu",
    state,ENABLE_MOTOR_OUTPUT?"LIVE":"DRY",armed?1:0,
    (long)goal[0],(long)goal[1],(long)pos[0],(long)pos[1],
    (long)vel[0],(long)vel[1],(long)torque[0],(long)torque[1],
    (unsigned long)(millis()-sampleAt),(unsigned long)millis());
  reply(out); // Cached data: STATUS never touches the motor bus or command timer.
}
void check() {
  if(ready || faulted) {reply("ERR CHECK_STATE");return;}
#if ENABLE_MOTOR_OUTPUT
  const char *log=nullptr;
  if(!dxl.init("",1000000,&log)) {fail("INIT");return;}
  for(uint8_t i=0;i<2;++i) {
    uint16_t model=0;
    if(!dxl.ping(IDS[i],&model,&log) || model!=1020 || dxl.getProtocolVersion()!=2.0f) {
      fail("IDENTITY");return;
    }
    known[i]=true;
    int32_t fw=0;
    if(!readItem(i,"Firmware_Version",fw) || fw<38 ||
       !eq(i,"Operating_Mode",1) || !eq(i,"Drive_Mode",0) ||
       !eq(i,"Homing_Offset",0) || !eq(i,"Torque_Enable",0) ||
       !eq(i,"Status_Return_Level",2) || !eq(i,"Bus_Watchdog",0)) {
      fail("PRECHECK");return;
    }
  }
#else
  pos[0]=3078;pos[1]=4096;
#endif
  if(!feedback())return;
  if(vel[0] != 0 || vel[1] != 0) {
    fail("SUPPORT_AT_NEUTRAL");
    return;
  }
#if ACCEPT_STATIONARY_POSE_AS_ORIGIN
  origin[0] = pos[0];
  origin[1] = pos[1];
#else
  if(labs(pos[0] - PAN_NEUTRAL) > NEUTRAL_TOL_COUNTS ||
     labs(pos[1] - TILT_NEUTRAL) > NEUTRAL_TOL_COUNTS) {
    fail("SUPPORT_AT_NEUTRAL");
    return;
  }
  origin[0] = PAN_NEUTRAL;
  origin[1] = TILT_NEUTRAL;
#endif
  for(uint8_t i=0;i<2;++i) {previous[i]=pos[i];holdAnchor[i]=pos[i];}
  baseline=true;ready=true;
  reply("CHECK_OK BOTH_TORQUES_OFF");
}
void hold() {
  if(!ready || holding || faulted) {reply("ERR HOLD_STATE");return;}
  // Still physically supported. Refuse a changed pose after CHECK.
  if(!feedback())return;
  if(labs(relativePosition(0,pos[0]))>NEUTRAL_TOL_COUNTS || labs(relativePosition(1,pos[1]))>NEUTRAL_TOL_COUNTS || vel[0]!=0 || vel[1]!=0) {
    fail("HOLD_POSE");return;
  }
#if ENABLE_MOTOR_OUTPUT
  // Verify both are off before clearing either watchdog or enabling either axis.
  if(!eq(0,"Torque_Enable",0)||!eq(1,"Torque_Enable",0) ||
     !eq(0,"Bus_Watchdog",0)||!eq(1,"Bus_Watchdog",0)) {fail("HOLD_PRECHECK");return;}
  if(!goals(0,0))return;
  for(uint8_t i=0;i<2;++i) {
    if(!verified(i,"Profile_Acceleration",PROFILE_ACCEL) || !verified(i,"Bus_Watchdog",BUS_TICKS)) {
      fail("HOLD_CONFIG");return;
    }
  }
  // Partial success is possible: on fault support before SUPPORTED_OFF.
  for(uint8_t i=0;i<2;++i) if(!verified(i,"Torque_Enable",1)) {fail("TORQUE_ENABLE");return;}
#endif
  holding=true;armed=false;
  for(uint8_t i=0;i<2;++i)holdAnchor[i]=pos[i];
  lastPoll=millis();
  stopBoth(true,"ACK HOLD ZERO_REQUESTED");
}
void stopBoth(bool disarm, const char *event) {
  if(disarm)armed=false;
  if(faulted)return;
  if(!holding) {goal[0]=goal[1]=0;reply(event);return;}
  if(!stopping) {stopAt=millis();quiet=0;stopping=true;}
  if(!goals(0,0))return;
  reply(event);
}
void off() {
  if(!ready && !faulted) {reply("ERR CHECK_FIRST");return;}
  // Explicit physical-support assertion. Can also release after a latched fault.
  bool ok=true;
#if ENABLE_MOTOR_OUTPUT
  for(uint8_t i=0;i<2;++i) {
    if(!known[i]) {ok=false;continue;}
    const bool one=verified(i,"Torque_Enable",0); ok=one&&ok;
  }
  if(!ok) {fail("OFF_UNCONFIRMED");reply("ERR OFF_UNCONFIRMED SUPPORT_AND_POWER_OFF");return;}
  for(uint8_t i=0;i<2;++i) {
    const bool one=verified(i,"Bus_Watchdog",0) && verified(i,"Goal_Velocity",0);
    ok=one&&ok;
  }
#endif
  armed=false;holding=false;stopping=false;goal[0]=goal[1]=0;torque[0]=torque[1]=0;
  // Require reset and a new CHECK after release. Do not reuse pose references.
  ready=false;baseline=false;faulted=true;
  reply(ok?"ACK SUPPORTED_OFF TORQUE_OFF_CONFIRMED RESET_REQUIRED":
           "ERR CLEANUP TORQUE_OFF_CONFIRMED POWER_OFF");
}
void timeout() {
  if(armed && (uint32_t)(millis()-lastCommand)>=COMMAND_MS)
    stopBoth(true,"EVENT TIMEOUT DISARMED ZERO_REQUESTED");
}
void service() {
  if(faulted)return;
  timeout();
  if(!holding || faulted)return;
  uint32_t now=millis();
  if((uint32_t)(now-lastPoll)<POLL_MS)return;
  lastPoll=now;
  if(!feedback())return;
  timeout();
  if(faulted)return;
  if(!stopping) {
    for(uint8_t i=0;i<2;++i) {
      int64_t d=relativePosition(i,pos[i]);
      if((d>=STOP_COUNTS[i] && goal[i]>0)||(d<=-STOP_COUNTS[i] && goal[i]<0)) {
        limitStop(i,goal[i]>0?1:-1);break;
      }
    }
  }
  if(stopping) {
    if(vel[0]==0 && vel[1]==0) {if(quiet<3)++quiet;} else quiet=0;
    if((uint32_t)(millis()-stopAt)>=100 && quiet>=3) {
      stopping=false;for(uint8_t i=0;i<2;++i)holdAnchor[i]=pos[i];
      reply("EVENT STOPPED TORQUE_RETAINED");
    } else if((uint32_t)(millis()-stopAt)>=500)fail("STOP_NOT_CONFIRMED");
  }
}
bool parsePair(const char *s, float &p, float &t) {
  char *end=nullptr;errno=0;p=strtof(s,&end);
  if(end==s||errno==ERANGE||!isfinite(p)||*end!=' ')return false;
  s=end+1;while(*s==' ')++s;
  errno=0;t=strtof(s,&end);
  return end!=s && *end=='\0' && errno!=ERANGE && isfinite(t);
}
void command(const char *s) {
  service(); // Expiry is checked before accepting any next command.
  if(!strcmp(s,"STATUS")){status();return;}
  if(!strcmp(s,"SUPPORTED_OFF")){off();return;}
  if(faulted){reply("ERR FAULT_RESET_REQUIRED");return;}
  if(!strcmp(s,"CHECK")){check();return;}
  if(!strcmp(s,"HOLD")){hold();return;}
  if(!strcmp(s,"DISARM")){stopBoth(true,"ACK DISARM ZERO_REQUESTED");return;}
  if(!strcmp(s,"STOP")) {
    if(armed)lastCommand=millis();
    stopBoth(false,"ACK STOP ZERO_REQUESTED");return;
  }
  if(!strcmp(s,"ARM")) {
    if(!holding||stopping){reply("ERR NOT_HOLDING");return;}
    if(armed){reply("ERR ALREADY_ARMED");return;}
    if(!feedback()||faulted)return;
    for(uint8_t i=0;i<2;++i)if(labs(relativePosition(i,pos[i]))>=ARM_POSE_TOLERANCE_COUNTS||vel[i]!=0) {
      reply("ERR ARM_POSE_OR_SPEED");return;
    }
    armed=true;lastCommand=millis();reply("ACK ARM");return;
  }
  if(!strncmp(s,"VEL ",4)) {
    float p=0,t=0;
    if(!parsePair(s+4,p,t)){reply("ERR VALUE");return;}
    if(fabsf(p)>MAX_RAD_S||fabsf(t)>MAX_RAD_S){reply("ERR RANGE");return;}
    if(!armed){reply("ERR DISARMED");return;}
    if(stopping){reply("ERR STOPPING");return;}
    const int32_t a=(int32_t)lroundf(p/RAD_S_PER_UNIT);
    const int32_t b=(int32_t)lroundf(t/RAD_S_PER_UNIT);
    // Entire pair validated before either write. Hardware writes are sequential.
    // Freshly poll feedback, then re-check timeout before enabling movement.
    if(!feedback())return;
    timeout();if(!armed||faulted||stopping){reply("ERR DISARMED");return;}
    const int32_t next[2]={a,b};
    for(uint8_t i=0;i<2;++i) {
      const int64_t d=relativePosition(i,pos[i]);
      if((d>=STOP_COUNTS[i] && next[i]>0)||(d<=-STOP_COUNTS[i] && next[i]<0)) {
        limitStop(i,next[i]>0?1:-1);return;   // no ACK VEL: the event answers this VEL
      }
    }
    lastCommand=millis(); // Bus latency counts toward 300 ms, not added afterward.
    const bool goingZero=(a==0&&b==0&&(goal[0]!=0||goal[1]!=0));
    for(uint8_t i=0;i<2;++i)if(next[i]==0&&goal[i]!=0)holdAnchor[i]=pos[i];
    if(!goals(a,b))return;
    if(goingZero){stopping=true;stopAt=millis();quiet=0;}
    reply("ACK VEL");return;
  }
  reply("ERR COMMAND");
}
void setup(){Serial.begin(115200);reply(ENABLE_MOTOR_OUTPUT?"READY MODE=LIVE BOOT":"READY MODE=DRY BOOT");}
void loop(){
  service();
  for(uint8_t n=0;n<32&&Serial.available()>0;++n){
    service();int c=Serial.read();if(c<0)break;
    if(c=='\r')continue;
    if(c=='\n') {
      if(discardLine)reply("ERR LINE");else if(used){line[used]=0;command(line);}
      used=0;discardLine=false;continue;
    }
    if(discardLine)continue;
    if(c<32||c>126||used>=sizeof(line)-1){discardLine=true;continue;}
    line[used++]=(char)c;
  }
  service();
}
