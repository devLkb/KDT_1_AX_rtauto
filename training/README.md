# DG5F grasp training

Canonical environment contract: [`docs/AGENT_SPEC.md`](../docs/AGENT_SPEC.md).
Architecture and rationale: [`docs/ML_AGENTS_DESIGN.md`](../docs/ML_AGENTS_DESIGN.md).
Step-by-step execution runbook: [`docs/ML_AGENTS_TRAINING_GUIDE.md`](../docs/ML_AGENTS_TRAINING_GUIDE.md).

- PPO config: `config/dg5f_grasp.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- generated builds/results are ignored by Git
- each Unity environment contains 20 independent training-area prefab instances
- default execution is CPU with one headless environment, for 20 agents total
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim
- `CONFIG`, `RESULTS_DIR`, `RUN_ID`, `ENV_PATH`, `NUM_ENVS`, and `TIME_SCALE` are overridable

Run a 50k-step smoke test by copying the config and changing `max_steps`, or override it with a temporary config. Do not commit checkpoints until the 100-episode evaluation gate passes.
