# DG5F grasp training

Current staged training plan: [`train_plan.md`](../train_plan.md).
The older documents under `docs/ML_AGENTS_*` describe the retired combined v4 reward.

- PPO config: `config/dg5f_grasp.yaml`
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

Shared-VDI convenience command (installed as `dg5f`):

```bash
dg5f status   # one-shot process/GPU/metric summary
dg5f watch    # continuously refresh the summary
dg5f logs     # follow logs
dg5f resume   # resume inside a shared tmux session
dg5f view     # attach to tmux read-only
dg5f stop     # graceful Ctrl+C/checkpoint
```
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

Current code is v1 reach-only training. Run a 50k-step smoke test by copying the config and changing `max_steps`. Promote to v2 only after at least 200 unseen-seed evaluation episodes reach 80% success, then start the new run with `--initialize-from dg5f_v1`.
