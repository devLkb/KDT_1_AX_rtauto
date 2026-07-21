# 문서 인덱스

## DG5FGraspPointReach 강화학습

- **처음 읽을 문서** — Unity·강화학습 용어부터 전체 흐름까지:
  [`ML_AGENTS_LEARNING_FLOW.md`](ML_AGENTS_LEARNING_FLOW.md)
- V2 Agent 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 외부 패키지 V3 계약: [`AGENT_SPEC_V3.md`](AGENT_SPEC_V3.md)
- 설계와 근거: [`ML_AGENTS_DESIGN.md`](ML_AGENTS_DESIGN.md)
- 빌드·학습 실행: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- V2 실패 학습 분석:
  [`V2_TRAINING_FAILURE_ANALYSIS_20260717.md`](V2_TRAINING_FAILURE_ANALYSIS_20260717.md)

읽는 순서:

1. [강화학습 입문과 전체 흐름](ML_AGENTS_LEARNING_FLOW.md)
2. [Agent 1.0.0 계약](AGENT_SPEC.md)
3. [환경·보상 설계](ML_AGENTS_DESIGN.md)
4. [구현 및 검증 로드맵](ML_AGENTS_ROADMAP.md)
5. [빌드·학습·평가 실행 가이드](ML_AGENTS_TRAINING_GUIDE.md)
6. [고정 학습 계획](train_plan.md)

제품 파이프라인은 `3D 카메라 좌표 -> RL 팔 이동 -> 텔레옵 손 조작`이다. 카메라는
후속 integration 경계로만 문서화되며 현재 학습 환경에는 수신 구현이 없다.

## DG5F 비전 텔레옵

- 시작점: [`../vision/dg5f/README.md`](../vision/dg5f/README.md)
- 보정: [`../vision/dg5f/CALIBRATION_GUIDE.md`](../vision/dg5f/CALIBRATION_GUIDE.md)
- 역할: 강화학습이 목표에 도달한 뒤 DG5F 손 20관절을 원격조작

텔레옵의 20관절 프로토콜은 유지되지만 강화학습 observation/action과는 독립이다.

## 이력과 진단

- [`WORKLOG.md`](WORKLOG.md): 프로젝트의 누적 작업 기록과 의사결정
- [`DEBUG_OSCILLATION_20260707.md`](DEBUG_OSCILLATION_20260707.md): 초기 진동 원인 분석
- [`DEBUG_OSCILLATION_20260708.md`](DEBUG_OSCILLATION_20260708.md): 관성 수정과 최종 검증

WORKLOG의 예전 단계형 파지 기록은 당시 이력이며 현재 실행 지침이 아니다. 활성 정책
계약은 항상 `AGENT_SPEC.md`를 우선한다.
