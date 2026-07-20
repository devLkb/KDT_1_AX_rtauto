# DG5F GraspPoint 팔 도달 강화학습 로드맵

## 확정 아키텍처

```text
3D 카메라 목표 좌표
  -> DG5FGraspPointReach (UR5e 팔 6축)
  -> DG5F 손 원격조작
```

강화학습 팀의 책임은 robot-base 목표 좌표로 `GraspPoint`를 빠르고 정확하게 이동하는
가운데 단계다. 카메라 수신과 손 파지는 각각 후속 integration과 기존 텔레옵 트랙에
속한다. move→grasp→lift 전이학습 계획과 손가락 20 action 정책은 폐기됐다.

## 1. 정책 계약 동결

- Behavior `DG5FGraspPointReach`, spec `1.0.0`
- observation 26, continuous arm action 6
- palm-local 단일 `GraspPoint`
- 20초 episode와 1 cm / 0.05 m/s / 0.25초 성공 조건

완료 조건은 C#, prefab, PPO YAML, 평가기가
[`AGENT_SPEC.md`](AGENT_SPEC.md)의 shape와 상수를 공유하는 것이다.

## 2. 독립 학습 환경

- 20개 독립 영역과 독립 seed
- 빨간 static-trigger 목표의 반경 0.20~0.85 m와 전 방위를 각각 균등 생성
- 패널 경계, 로봇 겹침, 너무 가까운 초기 목표 거부
- 학습 prefab에서는 손가락 물리·collider·수신 비활성
- reset 후 팔 상태, drive target, 속도와 reward 기억이 일치

완료 조건은 100회 이상의 reset에서 유효 목표와 유한 물리 상태가 유지되는 것이다.

## 3. 통신 및 안정성 검증

- EditMode에서 observation/action index, clamp, 목표 표본화, reward와 경계값 검증
- PlayMode에서 20개 영역 독립성, GraspPoint 고정 위치, 팔 6축 단독 제어 검증
- built player에 `max_steps: 512` smoke 설정을 적용해 communicator와 export 검증

smoke는 학습 성능이 아니라 communicator, tensor shape와 기본 reset 안정성의 gate다.

## 4. 단일 PPO 학습

checkpoint와 curriculum 없이 fresh run으로 시작한다. 기본 PPO는 5M steps,
batch 256, buffer 2048, learning rate `3e-4`, hidden 256×3, gamma `0.99`를 사용한다.
활성 문서와 launcher에는 단계 선택이나 이전 모델 bootstrap 경로를 두지 않는다.

## 5. 결정론 평가와 모델 선택

학습과 겹치지 않는 500 seed를 한 번씩 평가한다. 성공률 90% 이상과 정밀 성공 조건
전수 충족, 중복 seed/비유한 물리/안전 실패 0건을 모두 만족해야 한다. 통과 모델은
평균 최종 오차, median 완료 시간, p95 완료 시간 순으로 선택한다.

## 6. 후속 integration

카메라 adapter는 3D 카메라 좌표를 robot-base 미터 좌표로 보정해 target Transform에
주입한다. 이 경계에는 timestamp, 좌표 frame, stale-target 처리와 workspace 검증이
필요하지만 현재 학습 구현 범위에는 수신 transport를 넣지 않는다. 손은 목표 도달 후
`vision/dg5f` 텔레옵이 제어하며 policy I/O를 확장하지 않는다.
