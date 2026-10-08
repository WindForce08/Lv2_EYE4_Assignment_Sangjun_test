# Pan/Tilt 제어 인터페이스 (최종)

담당: 제어(조민혁) + 통합(한상준). 코드와 이 문서가 다르면 코드를 기준으로 이 문서를 고친다.
실행 위치: 모든 노드는 Raspberry Pi 한 대 (README 2절). 구현 상태와 장비 검증 상태는 [requirements_traceability.md](requirements_traceability.md)에 따로 표시한다.

## 1. 계층별 책임

| 계층 | 파일 | 책임 | 하지 않는 일 |
|---|---|---|---|
| perception_node | `detector.py`, `perception_node.py` | ex/ey/area_ratio, 미검출 z=0, 원본 영상 시각 | 거리로 검출 제한(선택 옵션 제외), 제어 |
| control_node | `control_core.py`, `control_node.py` | 입력 신선도, timeout 0.5 s, IDLE/TRACKING/LOST/SEARCHING, P 제어(상한 0.5 rad/s), **direction(유일)**, 축별 속도 상한·deadband, 3프레임 복귀, 목표가 없을 때 탐색 패턴, 경계 막힘 방향 차단 | 시리얼, 위치 적분, 각도 제한 |
| opencr_node | `serial_core.py`, `opencr_node.py` | 명령 나이 0.15 s, 0.5 rad/s 검사, DRY/LIVE 모드 확인, 명시적 prepare/arm/disarm, 보드 경계 정지 → 정지 확인 후 재개·`/opencr/limit` 발행, FAULT 래치 | 부호 변환, 자동 ARM, 자동 재연결 |
| 펌웨어 | `tracking_controller_2axis.ino` | rad/s→원시 단위, 0.5 rad/s 상한, 명령 대비 속도 감시, **엔코더 경계**(경계에서 두 축 정지·ARM 유지, 경계+60 counts FAULT), 명령 timeout 300 ms, Bus_Watchdog 200 ms, 고장 FAULT | 부호 변환, 자동 복구 |

각도 제한은 실제 엔코더 위치를 아는 펌웨어만 담당한다. control_node는 명령을 적분해 가짜 위치를 만들지 않는다.

## 2. `/target` (perception → control)

`geometry_msgs/msg/PointStamped`, QoS best effort · keep last 1 · volatile (구독도 best effort).

| 필드 | 의미 |
|---|---|
| header.stamp / frame_id | 입력 Color 영상의 header 그대로 (wrapper가 채운 영상 시각 — 촬영 시각인지는 미확인, Pi에서 확인 TODO; frame_id `camera_color_optical_frame`) |
| point.x | ex = (cx − W/2)/(W/2), 오른쪽 + |
| point.y | ey = (cy − H/2)/(H/2), 아래쪽 + |
| point.z | area_ratio = contour_area/(W·H), 검출 시 (0, 1], **미검출 = 0** |

PointStamped는 3D 위치가 아니라 이 과제의 목표 전달 규약이다. z에 Depth를 넣지 않는다. 영상마다 1번 발행하며 미검출도 z=0으로 발행한다.

## 3. 입력 판정과 상태 (control_core)

| 입력 | 판정 | 결과 |
|---|---|---|
| x,y,z 유한, \|x\|,\|y\|≤1, 0≤z≤1, stamp>0, 0 ≤ (now−stamp) ≤ 0.5 s, stamp가 직전보다 큼 | 신선·유효 | z>0이면 복귀 카운트 +1 / z=0이면 LOST |
| 위 조건 중 하나라도 실패 (NaN, 범위 밖, 오래됨, 미래, 중복·역순) | 무효 | LOST, 카운트 0 |
| 마지막 신선 입력 후 0.5 s (수신 단조 시각 또는 영상 시각 기준) | timeout | LOST |
| `/control/enable` false | 명시적 중지·탐색 취소 | IDLE, 입력 무시, 정지 |
| `/opencr/limit` `"pan:+1"` | 보드 경계 정지 (ARM 유지) | TRACKING → SEARCHING (두 번째면 IDLE(out_of_range)), SEARCHING → 탐색 반환점 |

### 3.1 목표가 없을 때 탐색 (SEARCHING, `search_enabled: true` — control.yaml)

| 상태 | 들어가는 조건 | 명령 | 나가는 조건 |
|---|---|---|---|
| LOST | 미검출·무효·timeout | 정지 | 검출 3프레임 → TRACKING. **신선한 미검출 프레임이 계속 오는 동안** `search_delay_sec`(3 s) → SEARCHING. 입력 자체가 끊긴 timeout에서는 탐색하지 않음 |
| SEARCHING | LOST 대기 끝, 또는 TRACKING 중 경계 정지(범위 초과) | 탐색 패턴, 원시 부호 ±`search_speed_rad_s` | 검출 3프레임 → TRACKING. 패턴 완료·`search_timeout_sec` → IDLE(search_done). timeout·무효 입력 → LOST(정지). `/control/enable false` → IDLE |
| IDLE(search_done) | 탐색 완료 | 정지 | 검출 3프레임 → TRACKING (탐색 반복 없음) |
| IDLE(out_of_range) | 경계로 다시 찾은 목표가 `track_stable_sec`(5 s) 안에 또 범위 초과 | 정지 (경계에 멈춘 자세) | 막힌 방향을 요구하지 않는(안쪽으로 돌아온) 목표 3프레임 → TRACKING |

탐색 패턴 — 각도 한계는 위치를 아는 펌웨어만 알고, control_node는 보드 경계 정지 이벤트를 반환점으로 쓴다:
1. LOCAL — 지금 Tilt 높이에서 Pan을 마지막으로 쫓던 방향 끝까지, 이어서 반대쪽 끝까지
2. TILT_EDGE — Tilt를 마지막으로 쫓던 방향 끝까지
3. SWEEP — Pan을 반대쪽 끝까지 한 줄 / 4. STEP — Tilt를 `search_tilt_step_rad`(0.5 rad, 시간 = step ÷ 속도)만큼 반대쪽으로 — 3·4 반복, 도중에 Tilt 반대쪽 끝에 닿으면 그 줄이 마지막
경계 이벤트가 오지 않으면(DRY) 한 방향 `search_leg_timeout_sec`(20 s)로 넘어간다. 경계에 막힌 축·방향(`edge_block`)으로는 명령하지 않고, 안쪽 명령이 0.5 s 이어지면 해제한다.
발제: 시야 밖 SEARCHING은 선택 심화(도전 B) — 시야 내 재등장 복구(가림 2 s × 5)와 통계를 분리해 기록한다. 그 시험은 `search_delay_sec`(3 s) 안에 재등장하므로 탐색이 끼어들지 않는다.

| 상태 | 명령 |
|---|---|
| IDLE (시작, 명시적 중지) | stop=true, 0, 0 |
| LOST (미검출, 무효, timeout) | stop=true, 0, 0 — 미검출 첫 프레임부터. LOST 전환 시 타이머를 기다리지 않고 즉시 발행 |
| TRACKING (신선 검출 3프레임 연속 후) | pan = clamp(pan_dir·kp_pan·ex, ±pan_limit), tilt = clamp(tilt_dir·kp_tilt·ey, ±tilt_limit), \|e\| ≤ deadband이면 그 축 0. 경계에 막힌 방향 성분은 0 |
| SEARCHING | 3.1절 탐색 패턴 (stop=false) |

복귀 카운트는 타이머가 아니라 새 영상 메시지 기준이다. timeout 판정을 복귀보다 먼저 하므로 발행이 끊겼다 재개된 첫 프레임으로 옛 TRACKING을 잇지 않는다.

## 4. `/control/pan_tilt_cmd` (control → opencr)

`realsense_tracker_interfaces/msg/PanTiltCommand` — `std_msgs/Header header`, `bool stop`, `float32 pan_velocity_rad_s`, `float32 tilt_velocity_rad_s`.
20 Hz 발행(QoS depth 1). 단위 rad/s, **모터 원시 부호**(direction 적용 후). stop=true면 두 축 0. `/tracking_status`(std_msgs/String)를 같은 주기로 발행.

direction 근거 (`docs/hardware.md`): Pan 원시 + = 좌측, Tilt 원시 + = 아래쪽 → 목표가 오른쪽(ex>0)이면 Pan −, 아래쪽(ey>0)이면 Tilt + → `pan_direction=-1`, `tilt_direction=+1`.

## 5. OpenCR bridge (opencr_node)

| 모드 | serial_mode | dry_run | expected_board_mode | enable_live_hardware | 설정 |
|---|---|---|---|---|---|
| DRY sink (포트 안 엶) | false | true | – | – | `control_dry.yaml` |
| 시리얼 DRY | true | true | DRY | false | `serial_dry.yaml` |
| 시리얼 LIVE | true | false | LIVE | true | `opencr_live.yaml` |

다른 조합은 시작 거부(`serial_core.check_mode`). 보드 STATUS의 MODE가 기대와 다르면 즉시 FAULT(명령 전송 없음).

세션: 연결 → STATUS(BOOT 확인, 기존 세션 인수 금지) → `/opencr/prepare`(CHECK→HOLD, 토크 ON 영속도) → READY → `/opencr/arm`(신선한 stop=false 명령이 있을 때만) → ARMED.
ARMED에서 신선한 명령마다 VEL 또는 STOP 1회 전송. stop=true(정상 목표 소실)는 STOP — ARM 유지, 다시 TRACKING이면 VEL 재개.
정지 중에는 VEL을 보내지 않고 20 ms STATUS로 `ARMED GOAL=0,0 VEL=0,0`을 확인한 뒤에만 재개한다 (이전 STOPPED 이벤트로 새 STOP을 해제하지 않음 — STOP 복귀 경쟁 조건 수정, `results/logs/control/serial_bridge_stop_fix_20261006_182228/`).

보드 경계 정지 `EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=PAN|TILT DIR=±1`(ARMED 중): FAULT가 아니다. 보드가 두 축을 멈추고 ARM을 유지했으므로
STOP과 같은 흐름 — 대기 중인 VEL의 응답으로 보고 대기를 해제, 정지 중 VEL 금지, STATUS로 `ARMED GOAL=0,0 VEL=0,0` 확인 후 재개.
경계와 엇갈려 보드가 처리한 그 VEL의 `ERR STOPPING` 1건은 무시(다음 STATUS 응답 전까지만). 축·방향은 `/opencr/limit`(std_msgs/String `"pan:+1"`, reliable depth 10)으로 발행.

FAULT(래치, 자동 재ARM·재연결 없음): ROS 명령 0.15 s 중단·오래된/역순/범위 밖 명령(ARM 중), ACK 0.2 s 미수신, STATUS 0.4 s 미수신, 보드 피드백 나이 >100 ms, 보드 재부팅,
EVENT TIMEOUT, EVENT LIMIT DISARMED(이전 펌웨어)·ARMED가 아닐 때의 EVENT LIMIT, FAULT/ERR(위 1건 제외), 깨진 응답, 시리얼 오류. FAULT 시 MODE가 확인된 보드에 DISARM 1회(best effort). 노드 종료 시 DISARM.
`/opencr/bridge_status`: phase, 최초 FAULT 원인, 보드 STATE 캐시, 수신 나이, 경계 정지 횟수·마지막 경계 (DRY의 위치·속도·토크는 모의 값).

## 6. 시리얼 프로토콜 (USB 115200 8N1, LF, 최대 63자)

| 명령 | 동작 | 응답 |
|---|---|---|
| STATUS | 캐시된 상태 (버스·타이머 영향 없음) | `STATE <s> MODE=<DRY\|LIVE> ARMED= GOAL=p,t POS=p,t VEL=p,t TORQUE=p,t AGE_MS= T_MS=` |
| CHECK | 모델 1020·Protocol 2.0·Operating Mode 1·토크 OFF·중립 자세 확인 | `CHECK_OK BOTH_TORQUES_OFF` |
| HOLD | 영속도·watchdog 설정 후 토크 ON | `ACK HOLD ZERO_REQUESTED` → `EVENT STOPPED TORQUE_RETAINED` |
| ARM | HOLD 완료·정지 상태에서 구동 허가, 300 ms 타이머 시작 | `ACK ARM` |
| VEL p t | 두 값 모두 검사 후 순차 쓰기, \|값\| ≤ 0.5 rad/s. 경계에서 바깥 방향이면 두 축 정지·ARM 유지 | `ACK VEL` / `EVENT LIMIT STOPPED ZERO_REQUESTED AXIS=… DIR=…`(ACK 대신) / `ERR RANGE` / `ERR DISARMED` / `ERR STOPPING` |
| STOP | 두 축 0, ARM 유지 | `ACK STOP ZERO_REQUESTED` |
| DISARM | 두 축 0, ARM 해제, 토크 유지 | `ACK DISARM ZERO_REQUESTED` |
| SUPPORTED_OFF | 카메라 지지 확인 후 토크 OFF, reset 필요 | `ACK SUPPORTED_OFF TORQUE_OFF_CONFIRMED RESET_REQUIRED` |

수락한 VEL·ARM 중 STOP만 명령 타이머를 갱신한다. STATUS·오류·거부 명령은 갱신하지 않는다.

## 7. Timeout·정지 계층

| 계층 | 감시 대상 | 값 | 동작 |
|---|---|---|---|
| control_node | 신선한 `/target` | 0.5 s (`target_timeout_sec`) | LOST, stop=true |
| opencr_node | 신선한 PanTiltCommand | 0.15 s (`command_max_age_sec`) | FAULT, DISARM 시도 |
| 펌웨어 | 마지막 수락 명령 | 300 ms (`COMMAND_MS`) | 두 축 0 + DISARM, 토크 유지 |
| 모터 | 버스 instruction packet | 200 ms (`Bus_Watchdog` 10 × 20 ms) | 모터 자체 정지 (보드 멈춤 대비) |

opencr_node는 마지막 비영 속도를 반복 전송해 보드 타이머를 갱신하지 않는다 (명령 1건 = 전송 1회). 300 ms는 timeout 설정값이지 실제 정지 시간이 아니다.
정상 정지는 영속도 + 토크 유지다. Tilt는 토크가 꺼지면 내려가므로 전원 상실·하드웨어 고장 시 자세 유지는 소프트웨어로 보장할 수 없다.
