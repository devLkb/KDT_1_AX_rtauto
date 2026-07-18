# DG5F grasp training

Current staged training plan: [`train_plan.md`](../docs/train_plan.md).
The older documents under `docs/ML_AGENTS_*` describe the retired combined v4 reward.

- v1 PPO config: `config/dg5f_grasp.yaml`
- v2 transfer config: `config/dg5f_grasp_v2.yaml`
- stable whole-hand transfer config: `config/dg5f_stable_grasp.yaml`
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

Current code implements spec 3.0 stable whole-hand grasp. Thumb contact (finger
index 0) plus at least two distinct non-thumb fingertips must remain simultaneous
for 0.5 simulation seconds before the ball is lifted 5 cm and held at or above
4 cm with linear speed at most 0.05 m/s for one second. `DG5FStableGrasp` keeps
the 116 observations and 26 continuous actions (arm 6 + hand 20) used by the
joint26 transfer policy.

Start a fresh stable-grasp run from the frozen 526647-step V1 checkpoint:

```bash
dg5f stable init
```

This command verifies the documented V1 SHA-256, creates a separate read-only
`dg5f_v1_stable_grasp_bootstrap`, and initializes the new
`dg5f_stable_grasp_gpu` run. It never resumes the V2 run. The source checkpoint
must first be restored at
`training/results/dg5f_v1_gpu_fixed/DG5FGrasp/checkpoint.pt`.

Start v2 from the frozen 526647-step v1 checkpoint:

```bash
dg5f v2 init
```

`init` first creates and verifies the read-only
`dg5f_v1_joint26_bootstrap`. It copies the V1 arm/task encoder columns, critic,
and six arm-action outputs, initializes the new hand inputs/outputs, drops
optimizer moments, and starts the new V2 global step at zero. The retired
closure run is preserved as `dg5f_v2_closure_failed_343k` and is rejected by
`resume`.

Resume only the v2 run:

```bash
dg5f v2 resume
```

After rebuilding the Linux player, evaluate one deterministic environment at real-time
scale. The Unity arguments allocate episode IDs and unique seeds across all 20 areas;
the validator rejects row/seed duplication, success holds below 0.5 seconds, non-finite
physics, or grasp/reach success rates below 80%.

```bash
DG5F_RUN_ID=dg5f_v2_joint26_gpu_fixed \
training/scripts/run_dg5f_v2_evaluation.sh
```

The frozen v1 hashes and local full-run archive are documented in
[`archives/V1_BASELINE.md`](archives/V1_BASELINE.md).

After training, rebuild `training/builds/DG5FStableGrasp`, run the deterministic
approval evaluation, and validate all 200 unseen seeds:

```bash
DG5F_RUN_ID=dg5f_stable_grasp_gpu \
training/scripts/run_dg5f_stable_evaluation.sh
```

The validator rejects any success without thumb + two non-thumb contacts
maintained after acquisition, a 5 cm lift, a one-second valid low-speed hold, or
the 80% stable-grasp and reach success rates. It also writes a model-bound
approval whose hashes tie the accepted ledger to that run's canonical ONNX.
Only then stage and assign the model:

```bash
vision/.vision/bin/python training/scripts/promote_dg5f_stable_model.py \
  training/results/dg5f_stable_grasp_gpu/DG5FStableGrasp.onnx \
  training/results/dg5f_stable_grasp_gpu/evaluation_stable.csv
```

In Unity, run `Tools/ML-Agents/Assign Approved DG5F Stable Model`. Both gates
update only `TrainingArea.prefab`; the current scene and existing model stay
untouched until the 200-episode approval is present.
