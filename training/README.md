# DG5F grasp training

Canonical environment contract: [`docs/AGENT_SPEC.md`](../docs/AGENT_SPEC.md).

- PPO config: `config/dg5f_grasp.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- generated builds/results are ignored by Git
- default execution is CPU with two parallel headless environments
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim

Run a 50k-step smoke test by copying the config and changing `max_steps`, or override it with a temporary config. Do not commit checkpoints until the 100-episode evaluation gate passes.
