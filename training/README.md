# DG5F grasp training

Current staged training plan: [`train_plan.md`](../train_plan.md).
The older documents under `docs/ML_AGENTS_*` describe the retired combined v4 reward.

- PPO config: `config/dg5f_grasp.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- generated builds/results are ignored by Git
- each Unity environment contains 20 independent training-area prefab instances
- default execution is CPU with one headless environment, for 20 agents total
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim
- `CONFIG`, `RESULTS_DIR`, `RUN_ID`, `ENV_PATH`, `NUM_ENVS`, and `TIME_SCALE` are overridable

Current code is v1 reach-only training. Run a 50k-step smoke test by copying the config and changing `max_steps`. Promote to v2 only after at least 200 unseen-seed evaluation episodes reach 80% success, then start the new run with `--initialize-from dg5f_v1`.
