# DG5F grasp training

Current staged training plan: [`train_plan.md`](../docs/train_plan.md).
The older documents under `docs/ML_AGENTS_*` describe the retired combined v4 reward.

- v1 PPO config: `config/dg5f_grasp.yaml`
- v2 hand-first transfer config:
  `config/dg5f_grasp_v2_handfirst_lr5e5.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- generated builds/results are ignored by Git
- each Unity environment contains 20 independent training-area prefab instances
- default trainer execution is CUDA with one Unity environment, for 20 agents total
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim
- the launcher uses the active virtualenv first, so run `vision` to activate `ax310` before training
- Unity 6000.4.0f1 crashes with `-nographics` on this Linux server; the launcher automatically uses Xvfb when `DISPLAY` is absent
- `VENV`, `TORCH_DEVICE`, `UNITY_DISPLAY_MODE`, `CONFIG`, `RESULTS_DIR`, `RUN_ID`, `ENV_PATH`, `NUM_ENVS`, and `TIME_SCALE` are overridable

GPU training example:

```bash
vision
ENV_PATH="$PWD/training/builds/DG5FGrasp/DG5FGrasp.x86_64" \
RUN_ID=dg5f_v1_gpu \
training/scripts/train_dg5f_grasp.sh
```

The startup line must show `[GPU] ... device=...`. In another terminal, `nvidia-smi`
should show the trainer Python process after PPO updates begin. The Unity simulator itself
runs through Xvfb; `--torch-device cuda` applies to the PPO neural network.

Shared-VDI convenience command (installed as `dg5f`) uses a stage-first CLI:

```bash
dg5f v2 status  # one-shot process/GPU/metric summary
dg5f v2 watch   # continuously refresh the summary
dg5f v2 logs    # follow logs
dg5f v2 resume  # resume the standard v2 run
dg5f v3 init    # initialize v3 from the standard v2 run
dg5f v4 stop    # trainer SIGINT, checkpoint, and verified shutdown
```

Each stage selects its standard run ID, config, player, and tmux session. `init`
automatically selects the previous stage (`v2 <- v1`, `v3 <- v2`, `v4 <- v3`).
An explicit source remains possible, for example `dg5f v3 init custom_v2_run`.
Stage-prefixed commands deliberately ignore stale exported stage variables. Custom
experiments use the legacy no-stage form with `DG5F_RUN_ID`, `DG5F_CONFIG`,
`DG5F_PLAYER`, and `DG5F_SESSION`.
Unity physics and environment stepping remain CPU workloads. With the default
`buffer_size: 10240`, CUDA utilization is bursty because PPO updates begin only after
enough trajectories have been collected; allocated CUDA memory already confirms that
the policy is resident on the GPU. Progress is printed every 200 agent steps.

`threaded: false` is intentional for ax310: its PyTorch default-device setting is
thread-local, while this ML-Agents version otherwise creates CPU trajectory tensors in
the background update thread and fails against CUDA model weights. The compatibility
launcher also initializes the selected device inside new Python threads, protecting old
smoke configs that still contain `threaded: true`. It selects PyTorch's legacy ONNX
exporter because ML-Agents pins ONNX 1.15.

Current code implements joint26 v2: thumb contact (finger index 0) and any
opposing finger (indices 1..4) must remain simultaneous for 0.5 simulation
seconds. V2's `DG5FGraspJoint` has 116 observations and 26 continuous actions
(arm 6 + hand 20). The packaged V3 behavior is renamed to
`DG5FStableGrasp`, but keeps the same tensor shapes.

Start v2 from the frozen 526647-step v1 checkpoint:

```bash
dg5f v2 init
```

`init` first creates and verifies the read-only
`dg5f_v1_joint26_bootstrap`. It copies the V1 arm/task encoder columns, critic,
and six arm-action outputs, initializes the new hand inputs/outputs, drops
optimizer moments, and starts the new V2 global step at zero. The retired
closure run is preserved as `dg5f_v2_closure_failed_343k` and is rejected by
`resume`. The joint26 pilots at learning rates `1e-4` and `5e-5` are preserved
as `dg5f_v2_joint26_gpu_fixed` and
`dg5f_v2_joint26_lr5e5_gpu_fixed`; neither can be resumed or selected for V3.
The former hand-first run
`dg5f_v2_joint26_handfirst3_lr5e5_gpu_fixed` stopped at step 839959 while the
result/player directories were being replaced and is now read-only. Operational
`dg5f v2` points to `dg5f_v2_stablegrasp_v3_lr5e5_gpu_fixed`: it starts at
step zero from the same verified joint26 bootstrap, without optimizer state,
and uses the StableGrasp 3.0.0 player at `5e-5`.

The hand-first curriculum locks all six arm targets during stage 1, leaves the
20 hand actions independent with at most 1 degree per decision, disables only
the approach potential, and uses a 5-second stage timeout. Stages 2 and 3 keep
the 20-second episode timeout, plus a distinct 5-second post-reach grasp
timeout. Curriculum transitions use the unsmoothed mean reward of the most
recent 200 episodes (`2.2` for stage 1 and `1.8` for stage 2).
Its rebuilt player is intentionally isolated at
`builds/DG5FGraspJoint26HandFirst`; the launcher will reject startup until that
new player has been built or uploaded, so it cannot silently use the retired
joint26 binary.

## Packaged V3 environment

`DG5FStableGrasp-v3.0.0-linux-x86_64-20260718.tar.xz` is installed separately
at `builds/DG5FGraspV3`; it must not replace the V2
`DG5FGraspJoint26HandFirst` directory. The package uses behavior
`DG5FStableGrasp`, curriculum parameter `stable_grasp_stage`, 116 observations,
and 26 continuous actions. `scripts/train_dg5f_grasp.sh` validates the installed
binary hashes and these trainer names using
`manifests/DG5FStableGrasp-v3.0.0.json` (or the identical manifest beside the
installed player).

The packaged environment is the current operational `v2` target:

```bash
dg5f v2 watch
```

Because ML-Agents resolves `--initialize-from` by behavior name, the launcher
creates an immutable local bridge from the bootstrap's
`DG5FGraspJoint/checkpoint.pt` to the package's
`DG5FStableGrasp/checkpoint.pt`. See
[`AGENT_SPEC_V3.md`](../docs/AGENT_SPEC_V3.md).

Resume only the v2 run:

```bash
dg5f v2 resume
```

After rebuilding the Linux player, evaluate one deterministic environment at real-time
scale. The Unity arguments allocate episode IDs and unique seeds across all 20 areas;
the validator rejects row/seed duplication, success holds below 0.5 seconds, non-finite
physics, or grasp/reach success rates below 80%.

```bash
DG5F_RUN_ID=dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed \
training/scripts/run_dg5f_v2_evaluation.sh
```

The frozen v1 hashes and local full-run archive are documented in
[`archives/V1_BASELINE.md`](archives/V1_BASELINE.md).
