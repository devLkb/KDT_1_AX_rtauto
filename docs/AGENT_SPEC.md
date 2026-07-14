# DG5FGrasp Agent Spec v1.0.0

## Goal and success

UR5e reaches a 4 cm red ball, DG5F grasps it, then lifts it at least 5 cm and maintains the grasp for 1 second.
Final evaluation passes at 80/100 successful episodes with median completion time at most 10 seconds.

Training uses `Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`. The original `DG5F_Import.unity` teleoperation scene remains unchanged.

## Interface contract

- Behavior: `DG5FGrasp`
- Policy: PPO
- Decision frequency: 10 Hz (`DecisionPeriod=5`, physics 50 Hz)
- Continuous actions: 7
- Vector observations: 43
- Maximum episode: 750 physics steps = 150 decisions / 15 seconds

### Actions

| Index | Meaning | Scale |
|---:|---|---|
| 0..5 | UR5e joint xDrive target delta | `[-1,1] * 2 deg/decision` |
| 6 | DG5F closure delta | `[-1,1] * 0.04/decision`, accumulated in `[0,1]` |

Arm targets are clamped to the training-safe ranges stored in `Dg5fGraspSpec`.
Closure linearly interpolates the validated left-hand `OPEN -> FIST` 20-joint profile, then clamps every target to its URDF drive limits.

### Observations

Order is immutable within spec v1:

| Range | Count | Value |
|---:|---:|---|
| 0..5 | 6 | normalized arm joint positions |
| 6..11 | 6 | normalized arm joint velocities |
| 12 | 1 | grip closure mapped to `[-1,1]` |
| 13..15 | 3 | ball position relative to GraspPoint in robot-base axes |
| 16..18 | 3 | ball linear velocity in robot-base axes |
| 19..21 | 3 | ball angular velocity in robot-base axes |
| 22 | 1 | ball lift relative to episode start |
| 23..37 | 15 | five fingertip positions relative to ball in palm axes |
| 38..42 | 5 | thumb/index/middle/ring/pinky contact flags |

### Reward and termination

- step cost: `-0.001`
- approach progress: up to `+/-0.15`
- fingertip enclosure progress inside 12 cm: up to `+/-0.10`
- first thumb + opposing finger contact: `+0.15` once
- lift progress: up to `+/-0.25`
- success: `+1.0`
- drop, workspace exit, or non-finite physics state: `-0.5`

Final success requires all conditions continuously for 1 second:

1. lift >= 5 cm;
2. thumb contact;
3. at least one opposing finger contact;
4. ball within 10 cm of GraspPoint.

## Curriculum and reset

1. **Reach**: kinematic ball, GraspPoint within 5 cm for 0.25 seconds.
2. **Grasp**: dynamic ball, thumb + opposing contact for 0.5 seconds.
3. **LiftAndHold**: final success definition above.

Each reset teleports all 26 revolute joints to the bundled start targets, clears joint velocity, opens the hand, clears contact state, and respawns the ball. Spawn half-width grows from 2 cm to 4 cm to 6 cm across lessons.

## Commands

```bash
# Python environment check
vision/.vision/bin/pip check
vision/.vision/bin/mlagents-learn --help

# Build headless environment from repository root
UNITY_EDITOR=/path/to/Unity/6000.4.0f1/Editor/Unity
"$UNITY_EDITOR" \
  -batchmode -nographics -quit -projectPath unity \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingBuild.BuildLinuxHeadless

# Editor-connected smoke training (press Play after trainer waits)
RUN_ID=dg5f_grasp_smoke TIME_SCALE=1 NUM_ENVS=1 \
  training/scripts/train_dg5f_grasp.sh

# Headless training
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_v1 NUM_ENVS=2 TIME_SCALE=10 \
  training/scripts/train_dg5f_grasp.sh
```

## Versioning rule

Any change to observation count/order, action meaning/scale, grip profile, or success contract increments this spec version. Existing checkpoints are incompatible until retrained.
