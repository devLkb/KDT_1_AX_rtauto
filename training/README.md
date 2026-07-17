# DG5F grasp training

Current staged training plan: [`train_plan.md`](../train_plan.md).
The older documents under `docs/ML_AGENTS_*` describe the retired combined v4 reward.

- v1 PPO config: `config/dg5f_grasp.yaml`
- v2 transfer config: `config/dg5f_grasp_v2.yaml`
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

Current code implements v2: thumb contact (finger index 0) and any opposing
finger (indices 1..4) must remain simultaneous for 0.5 simulation seconds. The
57-observation, 7-action, `DG5FGrasp` policy shape is unchanged from v1.

Start v2 from the frozen 526647-step v1 checkpoint:

```bash
dg5f v2 init
```

Resume only the v2 run:

```bash
dg5f v2 resume
```

After rebuilding the Linux player, evaluate one deterministic environment at real-time
scale. The Unity arguments allocate episode IDs and unique seeds across all 20 areas;
the validator rejects row/seed duplication, success holds below 0.5 seconds, non-finite
physics, or grasp/reach success rates below 80%.

```bash
DG5F_RUN_ID=dg5f_v2_gpu_fixed \
training/scripts/run_dg5f_v2_evaluation.sh
```

The frozen v1 hashes and local full-run archive are documented in
[`archives/V1_BASELINE.md`](archives/V1_BASELINE.md).
