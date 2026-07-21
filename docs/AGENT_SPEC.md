---
title: DG5FGraspJoint Agent Spec
version: 2.1.0
status: active
---

# DG5FGraspJoint Agent Spec 2.1.0

`Dg5fGraspSpec`과 `Dg5fGraspAgent`가 이 문서의 실행 기준이다. 이
계약은 V2, V3, V4에서 shape를 바꾸지 않는다.

## Policy

- Behavior: `DG5FGraspJoint`
- vector observations: 116, stack 1
- continuous actions: 26
- decision period: 5 physics steps
- stage 1 timeout: 5 simulation seconds
- stage 2/3 timeout: 20 simulation seconds, with a distinct 5-second
  post-reach grasp timeout (`MaxStep=0`)

## Observation order

| Range | Size | Value |
|---|---:|---|
| 0..5 | 6 | normalized arm joint positions |
| 6..11 | 6 | arm joint velocities |
| 12..31 | 20 | normalized hand joint positions |
| 32..51 | 20 | hand joint velocities |
| 52..71 | 20 | normalized hand xDrive command targets |
| 72..80 | 9 | ball relative position, linear velocity, angular velocity |
| 81 | 1 | ball vertical displacement |
| 82..96 | 15 | five fingertip positions relative to ball |
| 97..101 | 5 | thumb-to-pinky contacts |
| 102..107 | 6 | normalized arm xDrive command targets |
| 108..111 | 4 | V1/V2/V3/V4 objective one-hot |
| 112..115 | 4 | approach, best approach, threshold, elapsed time |

## Action order

- `0..5`: arm target-angle deltas
- `6..25`: finger 1..5, joint 1..4 target-angle deltas

Every hand action changes exactly one command target. The target accumulates by
at most `±4°` per decision and is clamped to that articulation xDrive's limits.
There is no closure scalar or hand-pose interpolation.

## Reset

All arm and hand articulation positions, velocities, xDrive targets, and the
Agent's target arrays are synchronized. Curriculum stages 1 and 2 reset the
hand to a fixed 35% pre-grasp pose; stage 3 resets it fully open.

The 20 training areas use independent balls, contacts, episode state, and
seeds while sharing the same policy.

## V2 reward and termination

- `-0.001` per decision
- stage 1: no approach-potential reward
- stage 2/3: `0.25 * delta(approach potential)`
- thumb-only contact potential `0.25`
- opposing-only contact potential `0`
- dual-contact potential `0.5`
- hold potential `0.5 * clamp(hold / 0.5 seconds)`
- dual contact held for 0.5 seconds: `+2.0`, success termination
- ball out of bounds or non-finite physics: `-1.0`, failure termination
- ordinary timeout or post-reach timeout: failure termination
- `Failure/Timeout` counts both timeout kinds;
  `Failure/PostReachTimeout` marks the post-reach subset

On any failed termination, remaining approach/contact/hold potentials are
settled back to zero. Losing dual contact resets the hold timer immediately
without ending the episode. Reaching 5 cm is a metric milestone, not a
termination condition.

## Curriculum

The standard ML-Agents environment parameter is `joint26_stage`.

1. ball within 4 cm of grasp point, fixed 35% pre-grasp, all arm targets locked
   to their initial pose, independent hand deltas capped at 1 degree
2. V1 0.35–0.70 m spawn distribution, normal arm, fixed pre-grasp
3. V1 spawn distribution, fully open hand, no control assistance

Lessons 1 and 2 advance on the unsmoothed mean reward of the most recent 200
episodes, at thresholds `2.2` and `1.8` respectively.

Deterministic approval evaluation always uses stage 3.

## Transfer

V1's 57x7 checkpoint cannot be loaded directly. The verified bootstrap tool
maps V1 encoder columns `0..11 -> 0..11` and `13..56 -> 72..115`, copies hidden
and critic/value tensors, and copies only the six arm action outputs. It drops
closure weights and optimizer moments, initializes the new hand tensors, and
sets the step to zero.

See [`train_plan.md`](train_plan.md) and
[`../training/README.md`](../training/README.md) for commands and gates.
