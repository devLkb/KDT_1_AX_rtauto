# DG5F hand-first V2 stale-build failed run

- Preserved run ID: `dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed`
- Final step: `417630`
- Policy contract: `DG5FGraspJoint`, 116 observations, 26 continuous actions
- Learning rate: `5e-5` with a linear schedule
- Initialization: verified `dg5f_v1_joint26_bootstrap`, global step zero
- Final checkpoint SHA-256:
  `c21005a52a0f083943e93d0ad92b08a5bd969a0817ba12f99a730396eed17cad`
- Final ONNX SHA-256:
  `43487b8ef2bcaae168e94289b84223d93ef8d3544df933aff9fcd28523e72236`
- Local result directory:
  `training/results/dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed`
- Local archive:
  `training/archives/dg5f_v2_joint26_handfirst_stalebuild_failed_417630.tar.gz`
- Local archive SHA-256:
  `4156610714579974c76c368c3c3ce6f442982b471eb4483ca0f738f20e52877f`

## Failure cause: the run never executed the hand-first environment

`run_logs/Player-0.log` shows the Unity player was loaded from
`training/builds/DG5FGraspJoint26/`, which was built at 2026-07-17 08:09,
before the hand-first changes to `Dg5fGraspAgent.cs` / `Dg5fGraspSpec.cs`
(2026-07-17 09:55). The `training/builds/DG5FGraspJoint26HandFirst/` player
that `dg5f.sh` expects had never been produced, so the run trained the old
arm+hand joint environment with the 20-second timeout instead of the
hand-first stage 1 (frozen arm, 5-second timeout, zero approach reward).

Evidence from the TensorBoard scalars:

- `Reach/FirstSuccessSeconds` averaged `6.29 s` in the 150k..200k window,
  which is impossible under the 5-second stage-1 timeout.
- `Environment/Episode Length` grew to ~198 decisions (~19.8 s at one
  decision per 0.1 s), matching the old 20-second timeout.
- `Environment/Lesson Number/joint26_stage` stayed `0` for all 417k steps.
  The 200-episode mean reward peaked near `1.76`, below the `2.2`
  lesson-1 threshold. Console lines such as "Mean Reward: 3.017,
  Std of Reward: 0.000" were single-episode samples caused by
  `summary_freq: 200` being far too small for ~80 concurrent agents.
- The policy then collapsed into a passive run-out-the-clock optimum:
  `Failure/Timeout` rose from 0.9% to 98.3%, `Grasp/Success` fell from
  63.6% to 1.7%, and `Failure/BallOutOfBounds` fell to 0% because not
  engaging the ball avoids the `-1` drop penalty while timeout costs 0.

## Corrective actions taken with the replacement run

1. `training/builds/DG5FGraspJoint26HandFirst/` was produced by hot-swapping
   a freshly compiled `KDT.GraspTraining.dll` (built with the Unity-bundled
   Roslyn compiler from the current sources) into a copy of the licensed
   2026-07-17 `DG5FGraspJoint26` player, because no Unity editor license is
   available on this machine. The class serialization layout was kept
   byte-identical to the baked scene (see the comments in
   `Dg5fGraspAgent.cs`); the swapped player loads with zero serialization
   layout warnings.
2. `training/scripts/train_dg5f_grasp.sh` now refuses to start when the
   player's `KDT.GraspTraining.dll` is older than any Grasp runtime source
   (`DG5F_SKIP_BUILD_FRESHNESS=1` overrides explicitly).
3. `summary_freq` was raised from `200` to `20000` so reported rewards are
   real multi-episode means.
4. The replacement run is `dg5f_v2_joint26_handfirst2_lr5e5_gpu_fixed`,
   initialized afresh from the frozen V1 joint26 bootstrap at step zero.

This is a failed run. It must not be resumed or selected as the V3 transfer
source.
