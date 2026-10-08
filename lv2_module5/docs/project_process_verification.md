# 프로젝트 프로세스 검증 결과 (2026-10-08)

> **대상**: `Final_assignment` 작업 트리, 브랜치 `dev/test1` @ `be2cf59` + 커밋 안 된 변경(bag 도구·문서 이동).
> **비교 대상**: 로컬에 마지막으로 fetch된 원격 브랜치(`origin/main`, `origin/integration/minhyeok-camera-dry-20261007_140322`, `origin/test2` 등). fetch 이후 원격 변경은 반영되지 않았다.
> **기준**: 발제 문서(문제 1~5, 6장 산출물, 7장 평가표 11행·역할 체크리스트 18문항, 제출 전 체크리스트).
> **방법**: 코드는 수정하지 않았다. 시험은 저장소 밖(임시 폴더)에서 실행했다. 이 PC에는 ROS 2와 실제 장비가 없다 → **ROS 노드 실행·colcon 빌드·실기 동작은 검증하지 못했다.** 대신 host 시험, PTY 시험, 저장된 원본 데이터 재계산, 모터 물리 모델을 붙인 폐루프 시뮬레이션으로 확인했다.
> **표기**: ✅ 확인됨(근거 있음) · ⚠️ 부분/조건부 · ❌ 미수행·불일치 · 🔴 제출·실기 차단 · 🟠 높음 · 🟡 중간 · 🟢 낮음

---

## 0. 요약

| 영역 | 판정 | 한 줄 |
|---|---|---|
| 소프트웨어 파이프라인 (인지 → 제어 → bridge → 펌웨어) | ✅ host 수준 | 단위 34·펌웨어 host DRY/LIVE_STUB·PTY 20회 반복·폐루프 시뮬레이션 모두 통과 |
| 인지 결과 재현성 (문제 1) | ✅ | 저장된 원본 43장에 현재 검출기 재실행 → 기록값과 일치 (최대 차 0.00005) |
| 실기 필수 시험 (문제 3·4·5) | ❌ | 이 브랜치 기준 Kp 6회, 30 s 정상 추적, 가림 5회, 중단 2종, bag 모두 미수행 |
| **브랜치 간 일관성** | 🔴 | 실기 LIVE 시연 증거는 `integration/minhyeok…` 브랜치에만 있고, **그 시연의 펌웨어·설정은 `dev/test1`과 다르다** (Tilt 방향 부호까지 반대) |
| 펌웨어 ↔ bridge 일관성 | 🔴 | 펌웨어는 경계에서 ARM 유지 정지, bridge는 같은 신호를 FAULT로 처리 → 세션 종료 |
| 설정·문서 일관성 | 🟠 | 속도 상한·경계·중립·Tilt 방향 값이 코드·hardware.yaml·README·report·presentation마다 다름 |
| 협업·제출 프로세스 (문제 5, 평가표 10·11) | ❌ | main은 10-06 PR #8에서 멈춤. 이후 작업은 PR 없이 개인·통합 브랜치에 분산. 리뷰·Issue 기록 없음, 태그 없음 |

**결론**: 코드 품질과 안전 계층 설계는 host 수준에서 일관되게 동작한다. 하지만 지금 상태로는 **어느 브랜치의 무엇을 제출·시연할지가 정해지지 않았고**, 이 브랜치로 LIVE를 바로 시도하면 Tilt 부호와 경계 처리에서 문제가 날 수 있다. 먼저 7절 1~4번을 처리해야 한다.

---

## 1. 실행한 검증과 결과

모든 명령은 `lv2_module5/` 기준이다. 산출물은 저장소 밖(임시 폴더)에 저장했다.

| # | 검증 | 명령 (요약) | 결과 |
|---|---|---|---|
| V1 | Python 단위 시험 | `cd ros2_ws/src/realsense_tracker && python3 -m unittest discover -s test` | ✅ **34/34 PASS** (control_core 9, detector 4, serial_core 21) |
| V2 | 펌웨어 host 시험 | `g++ … -DENABLE_MOTOR_OUTPUT=0/1 test_integrated_native.cpp` | ✅ `PASS … MODE=DRY`, `PASS … MODE=LIVE_STUB` |
| V3 | bridge ↔ DRY 펌웨어 PTY | `python3 -m realsense_tracker.test_serial_pty --cycles 20 --tick-sec 0.01` | ✅ `ALL SERIAL PTY CORE CHECKS PASSED` (5방향, STOP 후 재ARM 없이 복귀, 20회 STOP/복귀, 침묵 시 FAULT·자동 재ARM 없음) |
| V4 | 검출 재현성 | 저장된 `*_raw.png`에 `detector.detect()` + 현재 `tracker.yaml` 재실행 | ✅ 3장면 값 동일, visible 30장·empty 10장 노드 판정 40/40 일치, 최대 차 0.00005 (CSV 반올림 수준) |
| V5 | 폐루프 LIVE 시뮬레이션 | 실제 `Controller`(Kp 0.3, 상한 0.10, dir −1/−1) + `SerialBridge`(LIVE, 0.15 s) + **현재 LIVE 펌웨어** + 모터 물리 모델(Profile_Acceleration 반영, 위치 적분) | ✅ 목표 추종, 가림 2 s → 재등장 **0.11 s** 후 TRACKING, 느린 이동 추종(오차 ≈0.11), 제어 침묵 → `ROS_COMMAND_TIMEOUT`. 추적 중 펌웨어 FAULT 없음 |
| V6 | 경계 신호 통합 | 무장 상태 `SerialBridge`에 펌웨어의 `EVENT LIMIT STOPPED ZERO_REQUESTED` 입력 | ❌ bridge **FAULT** + DISARM 송신, 이후 `prepare` 불가("reset firmware first") — 9절 F3 |
| V7 | 정적 파싱 | Python 30개 AST, YAML 9개, bash 2개 `-n` | ✅ |
| V8 | 패키지 구성 | `setup.py` entry point 6개, launch·config 설치, `package.xml` 의존성 | ✅ |
| V9 | bag 도구 (미커밋) | 가짜 `ros2`·rclpy로 `bag_record.sh`·`bag_replay.sh`·`bag_tool.py` 전 경로 | ✅ 기록 종료(SIGINT 1회), 거부 조건 7종, A/B 재현, 대조 PASS/FAIL 판정 (실제 ROS에서는 미확인) |

**검증하지 못한 것**: colcon 빌드, ROS 노드 간 실제 DDS 통신·QoS, RealSense wrapper 연동, Pi 부하에서의 타이밍, 실제 모터 동작 전부. V5 시뮬레이션은 **장비 결과가 아니다** (카메라 지연·DDS·USB 지연·중력·기구 한계 없음).

---

## 2. 프로세스 단계별 검증

발제 3장의 흐름 *조건 확정 → 검출·인터페이스 → 제어 통합 → 성능·안전 → 재현·제출* 순서로 정리했다.

### STEP 1 — 범위와 조건 확정
| 항목 | 상태 | 근거 / 문제 |
|---|---|---|
| 대상·축·시험 조건 문서화 | ✅ | `config/test.yaml` (30 s, 2 s × 5, 3 s, 30/10 프레임) |
| Kp A/B 시험 전 확정 | ❌ | `config/test.yaml:24-28` 모두 `null`. `control.yaml:13` Kp 0.3은 "운영자 지정" |
| 실제 ID·baud·제한값 | ⚠️ | ID 11/12·1 Mbps·Protocol 2.0은 실측(`scan.log`). **현재 펌웨어 경계·원점은 근거 기록이 이 브랜치에 없음** (F4) |
| 4인 역할·Issue | ⚠️ | 역할은 README·team.md에 있음. Issue 링크 없음 |

### STEP 2 — 검출과 연결 (문제 1·2)
| 항목 | 상태 | 근거 |
|---|---|---|
| HSV·Contour 파이프라인, 정규화 오차, 면적비, 미검출 z=0 | ✅ | `detector.py`, 단위 시험, V4 |
| 정상·없음·가림 세 장면 | ✅ | `results/images/detection/*`, V4로 현재 코드에서 재현 |
| 검출률 30/30, 배경 오검출 0/10 | ✅ (조건부) | 사람 판정 CSV. 노트북 측정이고 Pi 측정은 없음. 프레임 분포가 치우침 (F10) |
| `/target` 규약 (PointStamped, best effort depth 1, header 복사) | ✅ 코드 | `perception_node.py`. 실제 DDS 연결은 미확인 |
| 모터 OFF 7개 모의 입력 | ⚠️ | 2026-10-06 Pi 기록은 **이전 commit·이전 설정**(Kp 0.1, 상한 0.05, Tilt +1). 현재 설정으로 재실행 기록 없음 |

### STEP 3 — 추적 통합 (문제 3)
| 항목 | 상태 | 근거 |
|---|---|---|
| P 제어·축별 포화·deadband·direction 한 곳 적용 | ✅ 코드 | `control_core.py`, 단위 시험 |
| 펌웨어 속도 상한 = 호스트 상한 | ✅ | 둘 다 0.10 rad/s (`.ino:49`, `control_core.py:32`) |
| 원시 방향 확인 | ⚠️ | 10-06 단일 명령 6케이스는 **재조립 전**. 이후 Tilt 방향 근거가 브랜치마다 상충 (F2) |
| 폐루프 오차 감소 부호 | ⚠️ | 통합 브랜치 LIVE 시연에서 작업자가 "예"라고 기록. 단 **그때 Tilt dir = +1**, 이 브랜치는 −1 |
| Kp 2종 × 3회 CSV·그래프 | ❌ | `results/plots/`는 `.gitkeep`만 있음 |

### STEP 4 — 성능·안전 (문제 4)
| 항목 | 상태 | 근거 |
|---|---|---|
| IDLE/TRACKING/LOST, 미검출 즉시 정지, 3프레임 복귀 | ✅ 코드·시뮬 | 단위 시험, V3, V5 |
| 정지 계층 (control 0.5 s → bridge 0.15 s → 펌웨어 300 ms → watchdog 200 ms) | ✅ 코드 / ⚠️ 실기 | 펌웨어 단독 300 ms는 10-06 실측. ROS 경유 실기 중단 시험은 미수행 |
| 30 s 정상 추적 (FPS·RMSE·유효 추적 비율) | ❌ | `results/logs/verification/`는 템플릿뿐 |
| 2 s 가림 5회 | ❌ | `recovery_trials.csv` 비어 있음. 통합 브랜치 시연의 "LOST→TRACKING 8회"는 가림 시각 기준이 없어 대체 불가 (해당 보고서 스스로 명시) |
| `/target` 중단, 제어 통신 중단 | ❌ 실기 | `interruption_trials.csv` 비어 있음 |

### STEP 5 — 재현과 제출 (문제 5)
| 항목 | 상태 | 근거 |
|---|---|---|
| bag 기록·재현 절차와 도구 | ⚠️ | `tools/bag_*.sh`, `bag_tool.py`, `config/bag.yaml` — **미커밋**, 실제 ROS 미검증 (V9) |
| 성공·소실 bag, 다른 팀원 재현 | ❌ | 기록된 bag 없음 |
| report.md / presentation.md | ⚠️ | 2026-10-07 기준. 현재 코드와 값이 다름 (F5). 실기 칸은 TODO |
| team.md 협업 증거 | ❌ | Issue·리뷰 링크 TODO, 보호 설정 미확인 (F8) |
| 제출 태그 `lv2-module5-submit` | ❌ | 로컬·원격 태그 없음 |

### STEP 6 — 시연
| 항목 | 상태 | 근거 |
|---|---|---|
| 실제 추적 시연 | ⚠️ | 통합 브랜치 `results/reports/live_200ms_20261008_133226_KO.md`: ARMED 약 146.7 s, FAULT 없음, Pan·Tilt 오차 감소(작업자 관찰). **이 브랜치 코드로는 미시연**, 영상은 아카이브에 미포함 |

---

## 3. 브랜치 상태 (로컬 fetch 기준)

| 브랜치 | main 대비 | 마지막 | 내용 |
|---|---:|---|---|
| `origin/main` | 0 | 10-06 | PR #1~#6, #8. 문서·초기 구조 |
| `dev/test1` (이 작업 트리) | +9 | 10-08 | 통합 정리(SeungHye-J), LIVE 안정화·원점 보정·경계 확대(WindForce08). **PR 없이 push** |
| `origin/integration/minhyeok-camera-dry-…` | +31 | 10-08 | 제어 담당 작업 전체 + **실기 증거**(LIVE HOLD, 200 ms LIVE 시연, 엔코더 중립 검사, 카메라 부하 DRY 60 s+). `dev/test1`에 미병합 |
| `origin/test2` | +4 | 10-08 | 정리 브랜치 (`dev/test1` 대비 파일 273개 변경, 약 75,000줄 삭제) |
| `origin/dev/{seunghye,sanghwa,sangjun,minhyeok}` | +10~+28 | 10-06~07 | 개인 작업 |

**같은 펌웨어 파일이 두 갈래로 갈라졌다**:

| 항목 | `integration/minhyeok` (LIVE 시연에 사용) | `dev/test1` (`be2cf59`) |
|---|---|---|
| 속도 상한 `MAX_RAD_S` / Kp / 호스트 상한 | 0.05 / 0.1 / 0.05 | 0.10 / 0.3 / 0.10 |
| `tilt_direction` | **+1** | **−1** |
| 원점 | 고정 Pan 3078, Tilt 0 mod 4096, CHECK 허용 ±50 | **CHECK 때 멈춰 있는 아무 자세** (`ACCEPT_STATIONARY_POSE_AS_ORIGIN 1`) |
| 경계 도달 | `EVENT LIMIT DISARMED` (DISARM) | `EVENT LIMIT STOPPED` (ARM 유지) |
| HOLD_DRIFT / ARM 허용 | 20 / 70 counts | 50 / 120 counts |
| bridge 명령 나이 | 0.20 s | 0.15 s |
| bridge 상태 발행 | 최소 100 ms 간격 (Pi 부하 대책, DRY 60 s+ 검증) | 10 ms마다 |

---

## 4. 발견 사항

### 🔴 F1. 실기 증거와 제출 후보 코드가 서로 다른 브랜치에 있다
- **사실**: 이 프로젝트의 유일한 폐루프 LIVE 증거(146.7 s, FAULT 없음)는 `integration/minhyeok`의 펌웨어·설정으로 얻었다. `dev/test1`은 그 뒤 다른 방향(원점 방식·속도·Tilt 부호·경계 처리)으로 바뀌었고, 바뀐 펌웨어의 실기 기록은 이 브랜치에 없다.
- **영향**: 발제 평가표 11번("제출 시점의 코드와 결과가 고정")과 report의 모든 실측 주장이 어느 코드에 대한 것인지 모호하다. 지금 `dev/test1`에 태그를 달면 **시연·증거와 다른 코드**를 제출하게 된다.
- **권장**: 팀장이 제출 기준 브랜치를 하나로 정하고, 나머지 증거를 PR로 합친다. 기준을 바꾸면 실기 시험을 그 코드로 다시 한다.

### 🔴 F2. Tilt 방향 부호의 근거가 서로 반대다 (안전)
- `dev/test1` `control.yaml:30` `tilt_direction: -1` — 근거는 주석("2026-10-07 재조립 후 실측")뿐이고 원본 로그가 없다.
- `integration/minhyeok`의 **2026-10-08 13:32 LIVE 시연**은 `tilt_direction: +1`로 "Tilt 동작이 수직 영상 오차를 줄였다"고 기록했다.
- `config/hardware.yaml:16` `tilt_positive: down`, 펌웨어 주석 `.ino:51` "tilt down"은 +1 쪽을 뒷받침한다.
- 이후 `3270103`(10-08 18:00)에서 중립 상수가 1043/2161로 바뀐 것은 **다시 조립했을 가능성**을 시사하지만 기록이 없다.
- **영향**: 부호가 틀리면 Tilt가 목표 반대쪽으로 돌아 목표를 잃거나 경계·기구 끝까지 간다(최대 0.10 rad/s).
- **권장**: LIVE 전에 README 12절 단계 A(단일 명령 방향)와 B(폐루프 부호)를 현재 조립 상태에서 다시 하고 `results/`에 원본을 남긴다. 그 결과로 `tilt_direction`과 문서를 맞춘다.

### 🔴 F3. 펌웨어 경계 정지와 bridge 처리가 맞지 않는다
- 펌웨어 `.ino:293`, `.ino:349`: 경계에서 `stopBoth(false, "EVENT LIMIT STOPPED ZERO_REQUESTED")` → ARM 유지, 안쪽 명령 허용(커밋 `be2cf59` 의도: "recover from reversible limit stops").
- bridge `serial_core.py:244`: `EVENT LIMIT`으로 시작하면 FAULT → DISARM 송신 → `prepare`는 BOOT에서만 가능 → **보드 reset 필요** (V6로 확인).
- host 펌웨어 시험(`test_integrated_native.cpp`)은 펌웨어만 보고 통과하므로 이 불일치를 잡지 못한다.
- **권장**: 제어·통합 담당이 한쪽으로 결정한다 — (a) bridge가 `EVENT LIMIT STOPPED`를 정상 정지로 받기, 또는 (b) 펌웨어를 DISARM으로 되돌리기. 결정한 쪽에 bridge + 펌웨어 통합 시험(PTY)을 추가한다.

### 🟠 F4. 원점이 "CHECK 때의 아무 정지 자세"이고 경계가 넓다 (안전)
- `.ino:61` `ACCEPT_STATIONARY_POSE_AS_ORIGIN 1`, `.ino:206` `origin = pos` → CHECK가 중립 자세를 더 이상 검사하지 않는다(속도 0만 확인). `PAN_NEUTRAL=1043`, `TILT_NEUTRAL=2161`(`.ino:53-54`)은 이 설정에서 쓰이지 않는다.
- 경계 `STOP_COUNTS = {1345, 662}`(약 ±118° / ±58°)가 **그 임의 자세 기준**으로 적용된다. 이전 수동 측정(재조립 전, 기구 한계 아님)은 Pan +404/−496, Tilt −372/+242 counts였다.
- **영향**: 카메라를 한쪽으로 돌린 상태에서 CHECK하면 소프트웨어 경계가 기구 끝을 넘을 수 있다. Tilt가 기구에 막히면 속도 0으로 버티며 과부하 오류가 날 때까지 힘을 준다.
- **권장**: 실제 기구 가동 범위를 측정·기록하고, 원점을 고정 기준(통합 브랜치 방식)으로 하거나 운영 절차로 "CHECK 전 중립 자세"를 강제한다. README 12·14절, `opencr_live.yaml` 주석의 "Pan 3078±20" 안내도 현재 동작과 다르다.

### 🟠 F5. 설정·문서 값 불일치

| 값 | 실제 (코드) | 다르게 적힌 곳 |
|---|---|---|
| 속도 상한 | 0.10 rad/s (`.ino:49`, `control_core.py:32`, `control.yaml:16-17`) | `.ino:9` 주석 0.05, `control.yaml:15` 주석 0.3, `config/hardware.yaml:19` 0.05, `README.md:55,110` 0.05, `config/README.md:22-23` 0.05, `docs/control_interface.md:12,13,80` 0.05, `report.md:101` 0.05, `presentation.md` ±0.05 |
| 경계 | ±1345 / ±662, 경계에서 ARM 유지 | `hardware.yaml:26` ±80, `README.md:110,243,410` ±80·DISARM, `report.md:99,122` ±80, `presentation.md:38` ±80, `docs/hardware.md:111` |
| 원점·중립 | CHECK 때 정지 자세 | `.ino:52` 주석 3078, `hardware.yaml:23` 3078, `README.md:212`, `firmware/opencr/README.md:48`, `opencr_live.yaml:5` "Pan 3078±20" |
| Tilt 방향 | `tilt_direction -1` (원시 + = 위) | `hardware.yaml:16` down, `.ino:51` down, `README.md:408`, `report.md:118`, `docs/control_interface.md:52` (+1) |
| Kp | 0.3 | `report.md`·`presentation.md`는 0.1 기준 |

`config/README.md`의 규칙("펌웨어 상수를 바꾸는 commit에서 hardware.yaml도 함께 수정")이 `3270103`·`be2cf59`에서 지켜지지 않았다.

### 🟡 F6. 속도 감시 여유가 1단위뿐이다
- `.ino:152` `vel > 5`이면 FAULT. 상한 0.10 rad/s는 원시 4단위(0.10 ÷ 0.02398 → 4)라서 여유가 1단위(≈0.024 rad/s)다. 통합 브랜치 시연(0.05 rad/s = 2단위)은 여유가 3단위였다.
- `Profile_Acceleration 1`(`.ino:234`)이 가속을 느리게 해 오버슈트를 줄이므로 시뮬레이션(V5)에서는 문제가 없었다. 실기에서 `FAULT UNEXPECTED_SPEED`가 나면 이 여유부터 의심한다.

### 🟡 F7. 비교 실험 조건이 정해지지 않았다
- Kp A/B가 `null`이고, 현재 Kp 0.3 / 상한 0.10이면 |ex| > 0.33부터 포화된다. 비교할 두 Kp 값은 포화 구간을 고려해 시험 전에 정하고 `test.yaml`에 commit한다.

### 🟡 F8. 협업 프로세스가 발제 요구와 다르다 (평가표 10)
- main에 병합된 PR: #1, #2, #4, #8 (SeungHye), #3, #5 (Han Sangjun), #6 (eiioitsMin). **김상화(SangHwaKim09)는 병합 PR 없음.**
- 10-07 이후 핵심 변경(`dev/test1` 9커밋, `integration/minhyeok` 31커밋)은 PR·리뷰 없이 브랜치에 있다. 발제는 "Issue → 브랜치 → PR → 리뷰 → 팀장 병합"을 요구한다.
- team.md의 Issue·리뷰 링크, main 보호 설정 확인, 대행 기록이 TODO다. 이 PC에서는 GitHub에 접속할 수 없어(`gh` 없음) PR 리뷰 기록 자체는 확인하지 못했다.

### 🟡 F9. 보고서·발표 자료가 현재 상태를 반영하지 않는다
- `report.md`는 2026-10-07 기준이다. 통합 브랜치의 10-08 실기 결과(LIVE HOLD, 200 ms 시연, DRY 60 s+, 중립 검사)가 반영되지 않았다.
- 7단계 작성 틀(구현·조건·결과물·측정·해석·심화·한계)과 성취도 번호 구성이 아니다.

### 🟢 F10. 평가 프레임 분포가 한쪽으로 치우쳤다
- visible 30장의 ex −0.50~+0.98, ey −0.78~+0.08. 사분면별: 오른쪽 위 19, 왼쪽 위 7, 오른쪽 아래 3, 왼쪽 아래 1. 발제의 "고르게 고른 최소 30프레임"을 설명할 때 이 분포를 함께 적는다.

### 🟢 F11. 영상 시각이 노드 시계보다 조금이라도 앞서면 LOST가 된다
- `control_core.py:97,121` `0 <= age` — 미래 시각을 전혀 허용하지 않는다. RealSense global time은 호스트 시계 추정값이다. 통합 브랜치 시연은 TRACKING을 유지했으므로 실제로 문제가 되지는 않았을 수 있다. 다만 간헐적 LOST가 보이면 이것을 확인한다.

### 🟢 F12. 호스트 쪽 FAULT 뒤에도 보드 reset이 필요하다
- `serial_core.py:184` `prepare`는 BOOT에서만 가능하다. 일시적인 `ROS_COMMAND_TIMEOUT`(V5에서 재현)에도 보드 reset → CHECK → HOLD를 다시 해야 한다. 안전 설계상 의도된 동작이지만, 시연 중 복구 시간을 고려해 절차를 연습한다.

### 🟢 F13. 커밋 안 된 변경이 있다
- bag 도구(`tools/bag_record.sh`, `bag_replay.sh`, `bag_tool.py`, `config/bag.yaml`), `eval_frames.py --output-dir`, README·recordings 문서, 문서 이동(`directory_workflow_guide.md`, `팀업무_네비게이터.md` → `docs/`)이 작업 트리에만 있다. PR로 올려 리뷰를 받는다.

---

## 5. 평가표 11행 대비 상태

| No. | 요구사항 | 상태 | 근거 / 막힌 점 |
|---|---|---|---|
| 1 | 목표·시험 조건 정의 | ⚠️ | test.yaml 있음. Kp A/B null (F7) |
| 2 | Pi·OpenCR 실행 환경 | ⚠️ | 빌드·업로드·시리얼 로그 있음(`results/logs/opencr`). OS 버전 미기록, 현재 펌웨어 업로드 기록 없음 |
| 3 | HSV·Contour 검출 | ✅ | V4 재현. Pi 측정 없음 |
| 4 | 인지·제어 인터페이스 | ⚠️ | 이전 설정으로 Pi DRY 기록. 현재 설정 재실행 필요 |
| 5 | P 추적·구동 제한 | ❌ | Kp 6회 없음. 경계 처리 불일치 (F3) |
| 6 | 목표 소실·복구 | ❌ | 가림 5회 없음 (시뮬레이션·시연 관찰만) |
| 7 | 통신 중단 안전 정지 | ⚠️ | 펌웨어 단독·PTY·시뮬레이션. ROS 경유 실기 없음 |
| 8 | 성능 측정·해석 | ⚠️ | 검출률·오검출·인지 FPS(노트북)만 |
| 9 | bag 기록·재현 | ❌ | 도구만 있음(미커밋) |
| 10 | 4인 협업·PR | ❌ | F8 |
| 11 | 최종 제출·시연 | ❌ | 기준 브랜치 미정 (F1), 태그 없음 |

## 6. 역할 체크리스트 18문항 요약

| 역할 | 충족 | 미충족 핵심 |
|---|---|---|
| 팀장 (1~4) | 범위·조건 문서화 | 보호 설정 기록, PR 경유 병합, 통합 확인·태그 |
| 인지 (5~7) | 파이프라인·세 장면·30/10 판정 | 병합된 본인 PR 없음 (17번) |
| 제어 (8~10) | ID·baud·방향(재조립 전), P 제어·포화·경계 구현, DRY 정지·복귀 | Kp 2종 × 3회, 실기 정지·복귀 (통합 브랜치에 일부 실기 관찰) |
| 통합 (11~13) | OpenCR 빌드·업로드·시리얼, 노드 구성 | 현재 코드 전체 연결의 Pi 실행 기록, bag 재현 |
| 검증·문서화 (14~16) | 조건·산식·도구 | 30 s·가림 5회·중단 시험 결과, report·presentation 갱신 |
| 공통 (17~18) | 3명 병합 PR | 4명 모두 리뷰 1건 이상, team.md 링크 |

---

## 7. 권장 조치 (순서대로)

1. **제출 기준 브랜치 결정** (팀장, F1) — `integration/minhyeok`와 `dev/test1` 중 하나를 기준으로 삼고, 다른 쪽의 필요한 변경과 증거(`results/reports/**`, bag 도구 등)를 PR로 합친다. 펌웨어 원점 방식·경계 동작·속도 상한·bridge 상태 발행 주기를 이때 하나로 정한다.
2. **Tilt 방향 재확인** (제어, F2) — 현재 조립 상태에서 단일 명령 방향(README 12절 A)과 폐루프 부호(B)를 시험해 원본을 `results/`에 남기고, `tilt_direction`·hardware.yaml·문서를 맞춘다.
3. **경계 처리 일치** (제어·통합, F3·F4) — 펌웨어와 bridge 중 한쪽으로 맞추고 PTY 통합 시험을 추가한다. 실제 기구 가동 범위를 재고 원점 방식을 정한다.
4. **설정·문서 동기화** (검증·문서화, F5) — 결정된 값으로 hardware.yaml·README·control_interface·report·presentation을 같은 PR에서 고친다.
5. **Kp A/B 확정 후 commit** (제어·검증, F7) → 실기 필수 시험: Kp 2종 × 3회, 30 s 정상, 가림 5회, `/target` 중단, 제어 통신 중단 (README 16·17절).
6. **bag 2개 기록과 다른 팀원 재현** (통합·검증) — `tools/bag_record.sh`, `tools/bag_replay.sh` (미커밋 도구를 먼저 PR로 올림).
7. **report.md 7단계 틀로 재작성**, presentation 갱신 (검증·문서화, F9).
8. **협업 증거 정리** (전원, F8) — 각자 본인 PR 병합 1건 이상과 타인 PR 리뷰 1건 이상, team.md 링크, main 보호 설정 확인 PR.
9. **최종 통합 확인** → `lv2-module5-submit` 태그 → 팀장 제출.

---

## 부록 A. 재현 방법

```bash
# V1 단위 시험 (ROS 불필요)
cd lv2_module5/ros2_ws/src/realsense_tracker && PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test
# V2 펌웨어 host 시험
cd lv2_module5/firmware/opencr/tests
g++ -std=c++17 -Wall -Wextra -Werror -I native_stubs test_integrated_native.cpp -o /tmp/nd && /tmp/nd
g++ -std=c++17 -Wall -Wextra -Werror -DENABLE_MOTOR_OUTPUT=1 -I native_stubs test_integrated_native.cpp -o /tmp/nl && /tmp/nl
# V3 PTY (DRY 펌웨어 실행 파일을 먼저 빌드)
g++ -std=c++17 -Wall -Wextra -I native_stubs native_pty.cpp -o /tmp/opencr_dry_pty
cd ../../../ros2_ws/src/realsense_tracker && python3 -m realsense_tracker.test_serial_pty \
  --firmware-executable /tmp/opencr_dry_pty --output-dir /tmp/pty --cycles 20 --tick-sec 0.01
# V4 검출 재현: results/logs/perception/log.csv, eval_*.csv의 raw_image에 detector.detect(cv2.imread(...), cfg) 재실행 후 값 비교
# V6 경계 신호: test/test_serial_core.py의 SerialTests.armed() 상태에서 line('EVENT LIMIT STOPPED ZERO_REQUESTED') → phase
```

V5 폐루프 시뮬레이션은 저장소 밖 임시 도구다(LIVE 펌웨어 + 가속·위치를 모델링한 `DynamixelWorkbench` 대체 헤더 + 실제 `Controller`·`SerialBridge`). 필요하면 별도 PR로 `firmware/opencr/tests/`에 추가할 수 있다.

## 부록 B. 이 검증의 한계

- ROS 2·RealSense·OpenCR·모터 없이 수행했다. 실기 동작, DDS QoS 호환, Pi 부하 타이밍, 기구 한계는 판단하지 않았다.
- 원격 브랜치 정보는 이 PC가 마지막으로 fetch한 시점 기준이다. GitHub PR·리뷰·보호 설정은 확인하지 못했다.
- 통합 브랜치의 실기 보고서는 그 브랜치의 문서 내용을 인용했다. 원시 아카이브(`*_evidence.tar.gz`)는 저장소에 없어 다시 계산하지 못했다.
