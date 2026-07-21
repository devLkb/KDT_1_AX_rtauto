# DG5F hand-first V2 second attempt: stopped at the 100k stage-1 gate

- Preserved run ID: `dg5f_v2_joint26_handfirst2_lr5e5_gpu_fixed`
- Final step: `107787`
- Policy contract: `DG5FGraspJoint`, 116 observations, 26 continuous actions
- Environment: `DG5FGraspJoint26HandFirst` (correct hand-first build,
  verified via `run_logs/Player-0.log` Mono path)
- Initialization: verified `dg5f_v1_joint26_bootstrap`, global step zero
- Final checkpoint SHA-256:
  `11b32a8f29ac309e09473e15131f9c9d8d225d5ebe159ac3bb5353cd78dc4a31`
- Final ONNX SHA-256:
  `1f8bd70087b84aff22ea9d1444794f7a938041fc336cb41c2dc58d4ab40f00de`
- Local archive:
  `training/archives/dg5f_v2_joint26_handfirst2_gate_stopped_107787.tar.gz`
- Local archive SHA-256:
  `f0e9b996d54e44e16dab4d838266a6f79791f8bbba500883e43d6bafa99bf8da`

## Why it was stopped

This was the first genuine test of the hand-first stage 1 (frozen arm,
5-second timeout, ball spawned at the grasp point). The documented gate
requires reaching stage 2 before 100k steps; at 100k the run was still in
lesson 0, so it was stopped per protocol. Unlike the stale-build failure,
the stage-1 mechanics were confirmed active (episode length ≤ ~32 decisions,
well under the 5-second cap).

The metrics show the failure was mechanical, not a slow learning curve:

| metric (per 20k summary) | 20k | 40k | 60k | 80k | 100k |
|---|---:|---:|---:|---:|---:|
| Grasp/Success | 0 | 0 | 0 | 0 | 0 |
| Grasp/DualContactReached | 0.000 | 0.002 | 0.000 | 0.000 | 0.000 |
| Grasp/MaxContactHoldSeconds | 0 | 0 | 0 | 0 | 0 |
| Failure/BallOutOfBounds | 0.58 | 0.46 | 0.48 | 0.46 | 0.45 |
| Failure/Timeout | 0.42 | 0.54 | 0.52 | 0.55 | 0.56 |

Zero grasp successes in ~3,200 episodes. Root cause: `ResetBall()` kept the
ball kinematic for only 2 fixed steps (0.04 s) before releasing it into free
fall. A falling ball clears the half-open hand in ~0.2-0.4 s, during which
the stage-1 rate limit of 1°/decision lets the fingers close only 2-4°.
Grasp success was therefore physically unreachable; reward "improvement"
(-0.60 → -0.47) was only the drop/timeout mix shifting.

## Corrective mechanics for the successor run

Stage 1 now holds the ball kinematic at the spawn pose until the hand
reaches thumb+opposing dual contact (or `StageOneBallHoldMaxSeconds = 2.5 s`),
then releases it under gravity; the dual-contact hold clock restarts at
release, so success (+2) still requires holding the dynamic ball for 0.5 s.
The stage-1 hand delta was raised to `2°/decision` via
`Dg5fGraspSpec.StageOneHandDeltaDegPerDecision` (the serialized agent field
is pinned to 1 by the baked scene and is no longer consulted). Successor
run: `dg5f_v2_joint26_handfirst3_lr5e5_gpu_fixed`.

This run must not be resumed or selected as the V3 transfer source.
