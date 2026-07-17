# DG5F closure V2 failed experiment

- Preserved run ID: `dg5f_v2_closure_failed_343k`
- Original run ID: `dg5f_v2_gpu_fixed`
- Final step: `343180`
- Policy contract: `DG5FGrasp`, 57 observations, 7 continuous actions
- Final checkpoint mean reward recorded by ML-Agents: `0.05009466230071017`
- Local result directory: `training/results/dg5f_v2_closure_failed_343k`
- Local archive: `training/archives/dg5f_v2_closure_failed_343k.tar.gz`
- Failure classification: the single closure action did not learn the required
  thumb/opposing-finger coordination, so this run is retained for comparison and
  must never be resumed or used as the joint26 transfer source.

SHA-256:

```text
10cfc93364bc543cee718ad140d1b1181d465a6a6882dade2235190b33eb48bd  DG5FGrasp/checkpoint.pt
7717609a9489810d6b1a4a73b4f8edc9e4c878fa9b0fe853b28253322edfc7bc  dg5f_v2_closure_failed_343k.tar.gz
```

The standard V2 run ID is now `dg5f_v2_joint26_gpu_fixed`. Its only valid
initialization source is the verified `dg5f_v1_joint26_bootstrap` run produced
from the frozen V1 checkpoint.
