# dg5f — DG5F 핸드트래킹 텔레옵 파이프라인

웹캠 MediaPipe → 20관절 각도 → UDP → Unity DG5F 구동. (SVH 파이프라인 `svh/`의 후속)

## 구성

| 파일 | 역할 |
|---|---|
| `dg5f_angles.py` | MediaPipe 21 landmark → 20채널 사람각도 → DG5F 관절각[deg] 매핑 (채널 테이블·보정 로드) |
| `vision_node_dg5f.py` | 웹캠 캡처 → 각도 → One Euro 필터 → UDP 송신 메인 루프 |
| `probe_sender.py` | 웹캠 없이 fist/open/cycle 패킷 송신 (배선 결정적 검증용) |
| `one_euro_filter.py` | 필터 (svh/에서 복사) |
| `analyze_teleop.py` | **텔레옵 추종 검증** — vision 로그 ↔ Unity 관절 로그 시간정렬 대조 (전송/클램프/추종/동작재현 4단계 분리 진단) |
| `dg5f_calibration.json` | (없으면 기본값) 채널별 human_min/max — 재보정 시 생성 |
| `CALIBRATION_GUIDE.md` | **보정 시 해야 할 동작 체크리스트** (촬영 조건 + 8가지 동작 + 문제해결) |

Unity 쪽: `Assets/Scripts/Dg5fReceiver.cs`(UDP 수신) + `Dg5fHandDriver.cs`(관절 주입).
4개 DG5F 프리팹 루트에 부착돼 있음 (`setup_drive.py --components`로 부착).

## 왼손 모델 사용

씬을 왼손으로 교체(Tools/DG5F/Left) 후, 스크립트에 `left` 인자:
```bash
python vision_node_dg5f.py left      # 웹캠에도 왼손을 보여줄 것
python probe_sender.py fist left
```
왼손 URDF는 일부 관절 리밋이 부호 반전(미러)돼 있어 `dg5f_angles.LEFT_MIRROR_CHANNELS`
채널을 자동 부호 반전: 엄지 4채널 전부 + 벌림 3채널 + 새끼 접기·MCP.
(엄지 mcp/ip·새끼 pip/dip 같은 대칭리밋 채널은 2026-07-13 왼손 주먹 프로브 시각 검증으로 확정
— 엄지 mcp/ip만 반전, 새끼 pip/dip은 반전 아님.) 보정(calibrate)은 좌우 공용 —
굽힘각 크기만 재므로 어느 손으로 보정해도 되지만, 실제 쓸 손으로 하는 게 정확.

## 프로토콜 (Python ↔ Unity 계약)

- UDP `127.0.0.1:5006` (⚠️ SVH는 5005 — 공존 가능)
- **v2 (현행)**: `'<24f'` = 관절각 20 + **엄지끝 정규화좌표 3 + 핀치 플래그 1**.
  v1(`'<20f'`, 관절각만)도 수신기가 하위호환으로 처리 (엄지 IK 비활성 → 채널 구동).
- 값 = **DG5F 관절공간 각도[deg]**. 사람→관절 매핑·보정·방향반전은 전부 Python 쪽.
  Unity는 URDF 리밋 clamp + lerp 스무딩(12/s) + xDrive.target 기록만.

## 엄지 손끝 위치 리타게팅 (v2, 2026-07-13)

채널별 선형 매핑은 엄지의 결합 운동(대향×굽힘×접기)을 재현 못함(OK 사인 끝 안 닿음,
손바닥 접기 소실) → **엄지만 관절각 대신 손끝 위치를 목표로 IK(순차 CCD, ArmTargetIK §18 패턴)**.
- 좌표 계약: 손바닥 해부학 좌표계(원점=중지MCP, ez=손목→중지MCP, ey=새끼→검지MCP,
  ex=cross) + |손목→중지MCP| 정규화. 해부학 랜드마크 기반이라 **좌/우·거울 불변**(미러 표 불필요).
  로봇 대응점: 손목=palm, 중지MCP=3_2, 검지MCP=2_2, 새끼MCP=5_3 (`Dg5fThumbIK`가 Start에서 캐시).
- **핀치 스냅**: 사람 엄지-검지 끝 거리 < 손크기×0.35 → 플래그 → 로봇 목표를 검지 끝
  (+1.2cm 접촉 오프셋)으로 전환. 비율 차이와 무관하게 접촉 보장.
- 검증: oktip 프로브 → 엄지-검지 끝 **1.2cm(=오프셋값) 정확 수렴**, OK 링 형성 스크린샷 확인.
  ⚠️ 검지 컬이 얕으면(OK 45/35/25°) 검지 끝이 엄지 작업공간 밖(최선 5.3cm) — OK 프로브는 62/52/38° 사용.
- ThumbIK 활성 시 Dg5fHandDriver는 엄지 4채널 주입을 건너뜀. `enableIK=false`로 레거시 복귀.
- 채널 순서(고정): `[0..3]` 엄지 1_1~1_4 / `[4..7]` 검지 2_1~2_4 / `[8..11]` 중지 /
  `[12..15]` 약지 / `[16..19]` 새끼. Unity는 `_dg_<손가락>_<마디>` **이름 접미사 매칭**
  (위치 매칭 금지 — SVH 근위/원위 뒤바뀜 함정 교훈. 접미사라 좌(ll_)/우(rl_) 모두 동작)

## 손가락/관절 의미 (2026-07-13 팁 좌표 실측으로 확정)

- 손가락 1 = **엄지**: 1_1 CMC굽힘[-22,77] / 1_2 **대향회전**[-155,0] / 1_3 MCP / 1_4 IP
- 손가락 2/3/4 = 검지/중지/약지: n_1 벌림 / n_2 MCP[0,115] / n_3 PIP / n_4 DIP
- 손가락 5 = **새끼** ⚠️구조 특이 (2026-07-13 관절 스윕 실측): 5_1 손바닥접기 / **5_2 측면 기울임
  (굽힘 아님! 굽힘성분 0.42 vs 측면 0.81 — 여기에 굽힘 넣으면 새끼가 옆으로 누움)** /
  5_3 굽힘(0.98) / 5_4 굽힘(0.99). 굽힘 관절 2개뿐 → 사람 MCP→5_3, (PIP+DIP)평균→5_4,
  5_1·5_2는 게이트 중립 0°.

## 검증 상태

- ✅ 배선 결정적 검증(2026-07-13, 웹캠 없이 probe_sender): fist/open 패킷 → 20관절
  xDrive.target 기대값 일치 **20/20, 오차 0.00°** (양방향), 실측각도 수렴 확인.
- ⬜ 라이브 웹캠 검증 (사용자): `python vision_node_dg5f.py` + Unity Play.
  확인 항목 ① 엄지 대향(1_2) 방향 — 반대면 `dg5f_angles.py`의 thumb_opp (dg_min,dg_max) 스왑
  ② 엄지 CMC(1_1) 방향 ③ 굽힘 6채널 자연스러움 → 필요 시 재보정.
- ✅ 보정 루틴 `calibrate_dg5f.py` (2026-07-13 작성) — **라이브 테스트 전 필수 실행**.
  펴기↔주먹·엄지대향·벌리기 각 3회+ → q → `dg5f_calibration.json` 자동 저장·자동 적용.
  ⚠️ 보정은 반드시 이 스크립트로 (vision_node 로그는 clamp된 값이라 역산 불가 — SVH 함정)
- ⬜ 게이트 해제: 벌림(n_1)·새끼접기(5_1)는 현재 **중립 0° 고정** (SVH spread 과민 전례).
  굽힘 검증 후 `dg5f_angles.py` DG5F_CHANNELS의 gated=False로 단계 해제.

## 주의

- Unity Play 중 같은 포트(5006)에 로컬 테스트 수신기 띄우지 말 것 (패킷 뺏김 — SVH 함정).
- IK/파지 수동 실험 시 `Dg5fHandDriver.enableTracking` 끄기 (매 프레임 덮어씀).
- vision 로그는 실행마다 새 파일 (`vision_dg5f_YYYYMMDD_HHMM.csv`) — truncate 함정 방지.
