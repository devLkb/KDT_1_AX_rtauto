# DG5F joint26 V2 learning-rate 5e-5 failed pilot

- Preserved run ID: `dg5f_v2_joint26_lr5e5_gpu_fixed`
- Final step: `215202`
- Policy contract: `DG5FGraspJoint`, 116 observations, 26 continuous actions
- Learning rate: `5e-5` with a linear schedule
- Initialization: verified `dg5f_v1_joint26_bootstrap`, global step zero
- Final checkpoint SHA-256:
  `8ed86d272f56d4341034af802fca2ce53605de14a96ee5db8baa30d046423057`
- Final ONNX SHA-256:
  `ec8a84a2f59cfcf81446b7bf3d20185967147c55b5345b845bdf95f5a56d547e`
- Local result directory:
  `training/results/dg5f_v2_joint26_lr5e5_gpu_fixed`
- Local archive:
  `training/archives/dg5f_v2_joint26_lr5e5_failed_215202.tar.gz`
- Local archive SHA-256:
  `dc0c4b5920dea2c499adb7e666ae741098b9220067f1c96415b77bfceefa9822`

The trainer received `SIGINT`, wrote the final checkpoint, exported the ONNX,
and all trainer and Unity Player processes stopped. ML-Agents logged worker
shutdown cleanup errors after export, but there were no training-time
communicator or non-finite-physics errors.

At the latest complete scalar summary (step 215000), cumulative reward was
`1.3623`, grasp success `0.333333`, dual-contact reached `0.333333`, reach
success `0.333333`, ordinary timeout `0.333333`, ball out of bounds `0.333333`,
and non-finite physics `0`. These point samples are not a performance estimate
on their own, but the run remained in lesson 1 after 200k steps and reproduced
the arm/hand learning competition and long-timeout sampling problem.

This is a failed comparison run. It must not be resumed or selected as the V3
transfer source. The replacement run is
`dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed`, initialized afresh from the frozen
V1 joint26 bootstrap at step zero without optimizer state.
