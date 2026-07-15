# DG5F grasp training

Canonical environment contract: [`docs/AGENT_SPEC.md`](../docs/AGENT_SPEC.md).
End-to-end episode and PPO flow: [`docs/ML_AGENTS_LEARNING_FLOW.md`](../docs/ML_AGENTS_LEARNING_FLOW.md).
Architecture and rationale: [`docs/ML_AGENTS_DESIGN.md`](../docs/ML_AGENTS_DESIGN.md).
Step-by-step execution runbook: [`docs/ML_AGENTS_TRAINING_GUIDE.md`](../docs/ML_AGENTS_TRAINING_GUIDE.md).

- PPO config: `config/dg5f_grasp.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- exact lesson gate: `scripts/promote_dg5f_lesson.py`
- generated builds/results are ignored by Git
- each Unity environment contains 20 independent training-area prefab instances
- default execution is CPU with one headless environment, for 20 agents total
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim
- `CONFIG`, `RESULTS_DIR`, `RUN_ID`, `ENV_PATH`, `NUM_ENVS`, and `TIME_SCALE` are overridable

Run a 50k-step smoke test by copying the config and changing `max_steps`, or override it with a temporary config. Lesson promotion requires at least 200 evaluation episodes at 80% success; final selection uses 500 unseen seeds and requires 90% success.
