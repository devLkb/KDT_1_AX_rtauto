# DG5FGraspPointReach 강화학습 입문과 전체 흐름

## 1. 무엇을 학습하는가

이번 정책이 배우는 것은 하나다. 빨간 목표의 3차원 좌표를 받으면 UR5e 팔을 움직여
손바닥의 `GraspPoint`를 가능한 한 빠르고 정확하게 그 좌표에 놓는다.

```text
현재 학습: 랜덤 목표 좌표 -> 강화학습 팔 이동
전체 제품: 3D 카메라 좌표 -> 강화학습 팔 이동 -> 원격 손 조작
```

손가락을 쥐거나 물체를 들어 올리는 행동은 학습하지 않는다. DG5F 손 20관절은 이후
사람이 텔레옵으로 조작한다.

## 2. Unity와 Python의 역할

Unity는 로봇 물리, 목표 생성, 관측 계산, 행동 적용, 보상과 episode 종료를 담당한다.
Python의 ML-Agents PPO trainer는 Unity가 보낸 경험으로 neural network를 업데이트하고
ONNX 모델을 만든다.

한 번의 decision은 다음 순서다.

1. Unity가 현재 팔과 목표 상태 26개를 보낸다.
2. policy가 팔 관절 6개의 연속 행동을 반환한다.
3. Unity가 행동을 관절 target 증분으로 적용하고 물리를 진행한다.
4. 목표에 가까워졌는지, 빠르고 안정적으로 도달했는지 보상한다.

## 3. Agent가 보는 26개 값

- 팔 관절각 6개
- 팔 관절속도 6개
- 현재 관절 drive 명령 6개
- GraspPoint에서 목표까지의 3차원 오차 3개
- 목표까지 거리 1개
- GraspPoint 선속도 3개
- 성공 조건 유지 진행도 1개

합계는 26개다. 목표의 카메라 픽셀이나 손가락 관절 상태는 포함하지 않는다. 카메라
시스템은 나중에 robot-base 좌표로 변환한 결과만 Unity target에 전달한다.

## 4. Agent가 하는 6개 행동

행동 6개는 UR5e 6관절에 일대일 대응한다. `-1..1` 출력은 decision당 최대 ±4도의
명령 증분으로 바뀌고 관절 안전 범위에서 clamp된다. 손가락 20개는 policy action이
아니며 학습 환경에서 구동되지 않는다.

## 5. 한 episode

1. 팔 상태와 속도를 초기화한다.
2. 빨간 공의 반경 0.20~0.85 m와 방위 360°를 각각 균등 표본화한다.
3. Agent가 최대 20 simulation seconds 동안 GraspPoint를 이동한다.
4. 1 cm 이내이면서 0.05 m/s 이하인 상태를 0.25초 유지하면 성공한다.
5. 성공, timeout, workspace 이탈 또는 비유한 물리 상태에서 episode를 끝낸다.

20개 학습 영역은 서로 다른 seed로 이 과정을 병렬 실행한다.

## 6. 보상이 유도하는 행동

목표에 가까워진 거리만큼 양의 보상, 멀어진 거리만큼 같은 크기의 음의 보상을 받는다.
매 decision에는 `-0.001` 시간 비용이 있어 같은 정밀도라면 더 빠른 움직임이 유리하다.
성공 보너스는 남은 시간과 최종 오차를 반영한다.

빠르게 통과만 하면 성공이 아니다. 목표 근처에서 속도를 낮춰 0.25초 안정적으로
머물러야 한다. 접근과 후퇴를 반복하면 거리 보상이 상쇄되고 시간 비용만 쌓이므로
reward farming이 되지 않는다.

## 7. PPO 학습 흐름

1. 512-step smoke로 Unity-Python 통신과 26/6 tensor shape를 확인한다.
2. 새 run ID로 checkpoint 없이 PPO 학습을 시작한다.
3. TensorBoard에서 reward, episode length, 성공률과 물리 실패를 관찰한다.
4. 최대 5M steps까지 학습하고 checkpoint를 보존한다.
5. 학습에 쓰지 않은 500 seed로 결정론 평가한다.

이전 move→grasp→lift 단계, curriculum, checkpoint 전이는 사용하지 않는다.
상세 명령은 [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md), 정확한
정책 계약은 [`AGENT_SPEC.md`](AGENT_SPEC.md)를 참고한다.

## 8. 합격 기준

- 500개 seed 성공률 90% 이상
- 성공한 모든 episode가 1 cm / 0.05 m/s / 0.25초 조건 충족
- 중복 seed, NaN/Infinity, workspace 안전 실패 없음
- 통과 모델 중 평균 최종 오차가 가장 작고, 동률이면 median/p95 완료 시간이 가장 짧음

smoke가 끝났거나 평균 reward가 올랐다는 사실만으로 모델을 승인하지 않는다.
