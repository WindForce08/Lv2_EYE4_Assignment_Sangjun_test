# verification logs (문제 3·4·5)

| 파일 | 만드는 방법 | 내용 |
|---|---|---|
| `<run_id>.csv` | `tools/tracking_logger.py --run-id <run_id>` | /target 프레임마다 ex·ey·면적비·상태·명령 (실시간) |
| `<run_id>_reanalysis.csv` | `tools/bag_replay.sh reanalysis` | 문제 5 B: bag의 저장된 /target·상태·명령을 같은 기록기로 다시 기록 |
| `<run_id>_reprocess.csv` | `tools/bag_replay.sh reprocess` | 문제 5 A: bag 영상을 검출기에 다시 넣은 `/target_replay` (상태·명령 열은 비어 있음) |
| `<run_id>_serial.csv` | opencr_node `csv_path` (Pi `~/runs/`에서 복사) | 같은 실행의 시리얼 송수신. Pi 단조 시계 — bag 시각과 빼서 지연을 계산하지 않음 |
| `recovery_trials.csv` | 사람이 영상·CSV를 보고 작성 | 2초 가림 5회: 가림 시작·재등장·TRACKING 복귀 시각, 복구 시간, 성공 여부 |
| `interruption_trials.csv` | 사람이 시리얼 CSV·관찰로 작성 | `/target` 중단, 제어 통신 중단: 중단 시각, 정지 확인 시각, 정지 주체(control/bridge/board) |

- `run_id`는 bag 이름·시리얼 CSV(`opencr_node csv_path`)·이 CSV에 똑같이 쓴다. 예: `kpA_01`, `occlusion_01`, `success_01`.
- 재등장 시각은 사람이 영상으로 판정한다. `analyze_tracking.py`의 "첫 재검출"은 검출 기준 보조값이다.
- 복귀하지 못한 회차는 `recovery_sec`를 비우고 `success=0`으로 적는다 (0초로 쓰지 않음). 실패 회차도 지우지 않는다.
- 2026-10-07 현재 두 템플릿은 비어 있다: 실제 장비 시험 전이다.
