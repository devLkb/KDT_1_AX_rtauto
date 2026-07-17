# DG5F v1 frozen transfer baseline

- Source run: `dg5f_v1_gpu_fixed`
- Frozen step: `526647`
- Archived on: `2026-07-17`
- Local archive: `training/archives/dg5f_v1_gpu_fixed_526647.tar.gz`
- Transfer input: `training/results/dg5f_v1_gpu_fixed/DG5FGrasp/checkpoint.pt`

SHA-256:

```text
75f5b5a6c88601b90fb9fb44e21d883a48df7ed3f6e8a23d4bba80f82768e066  DG5FGrasp/checkpoint.pt
d5ea4fe93b4d88e48baa646bfdafb0dc7d24b35505d5e18f8ff1447aaa9501a3  DG5FGrasp/DG5FGrasp-526647.onnx
d5ea4fe93b4d88e48baa646bfdafb0dc7d24b35505d5e18f8ff1447aaa9501a3  DG5FGrasp.onnx
99fee768b580bb4b3dc816d404d6d26dca18a71d5b149e9332b0a8b9b19af509  dg5f_v1_gpu_fixed_526647.tar.gz
```

`run_logs/training_status.json` records both the final checkpoint and exported ONNX
at step `526647`. Never use `--resume`, `--force`, or the v2 run ID against this source
directory.
