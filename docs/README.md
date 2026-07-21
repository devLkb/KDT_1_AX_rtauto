# 문서 인덱스

## DG5F 강화학습

- **처음 읽을 문서** — Unity·강화학습 용어부터 전체 흐름까지:
  [`ML_AGENTS_LEARNING_FLOW.md`](ML_AGENTS_LEARNING_FLOW.md)
- V2 Agent 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 외부 패키지 V3 계약: [`AGENT_SPEC_V3.md`](AGENT_SPEC_V3.md)
- 설계와 근거: [`ML_AGENTS_DESIGN.md`](ML_AGENTS_DESIGN.md)
- 빌드·학습 실행: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- V2 실패 학습 분석:
  [`V2_TRAINING_FAILURE_ANALYSIS_20260717.md`](V2_TRAINING_FAILURE_ANALYSIS_20260717.md)

## SVH 비전 파이프라인

카메라 → MediaPipe → 삼각측량 → SVH 9관절 각도 → 필터 → UDP → 유니티

## 파일
- `one_euro_filter.py` — 실시간 노이즈 억제 필터 (관절 떨림 = 모터 진동 방지)
- `svh_angles.py` — landmark 21점 → SVH 9채널 각도 변환 + 매핑
- `vision_node.py` — 메인 루프 (캡처·검출·삼각측량·필터·UDP 전송)
- `SvhReceiver.cs` — 유니티 수신 스크립트

## 반드시 직접 채워야 하는 부분 (placeholder)
1. **스테레오 캘리브레이션** `stereo_calib.npz`
   체커보드를 두 카메라로 동시 촬영 → `cv2.calibrateCamera`(각각) →
   `cv2.stereoCalibrate` → mtxL, distL, mtxR, distR, R, T 저장.
   이게 없으면 삼각측량이 불가능합니다.
2. **SVH 관절 범위** `svh_angles.py`의 `SVH_CHANNELS`
   각 채널 (svh_min, svh_max)를 SVH URDF/매뉴얼 실제 rad 값으로 교체.
   지금은 임시값이라 실물에 그대로 보내면 안 됩니다.
3. **사람 손 각도 범위** 같은 표의 (human_min, human_max)
   본인 손을 폈다/굽혔다 하며 raw 값을 출력해 보고 보정.
4. **카메라 인덱스** `vision_node.py`의 CAM_LEFT / CAM_RIGHT.

## 실행 순서 (권장)
### 1단계: 단일 카메라로 뒷단 검증
`vision_node.py`에서 `STEREO = False`.
카메라 1대만으로 각도 계산·필터·UDP·유니티 수신이 맞는지 확인.
(z는 부정확하지만 파이프라인 검증엔 충분)

### 2단계: 스테레오로 확장
캘리브레이션 파일 준비 후 `STEREO = True`.
입력만 바뀌고 뒷단(각도·필터·전송·유니티)은 그대로 재사용됩니다.

## 안전
- 실물 SVH 전에 **MuJoCo 시뮬에서 먼저** 검증 (드라이버에 시뮬 포함).
- 프레임 간 각도 변화량 상한(rate limit)을 SVH 전송 직전에 추가 권장.
- occlusion 시 값 튐 방지를 위해 마지막 유효값 hold가 이미 들어가 있음.

## 확장: 비전을 다른 기기로 분리
노트북이 벅차면 `vision_node.py`를 별도 PC에서 돌리고
`UNITY_IP`를 유니티 노트북의 IP로 바꾸면 됩니다. 나머지는 동일.
