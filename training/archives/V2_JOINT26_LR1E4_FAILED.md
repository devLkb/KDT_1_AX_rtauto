# DG5F joint26 V2 learning-rate 1e-4 failed pilot

- Preserved run ID: `dg5f_v2_joint26_gpu_fixed`
- Final step: `253740`
- Policy contract: `DG5FGraspJoint`, 116 observations, 26 continuous actions
- Learning rate: `1e-4` with a linear schedule
- Initialization: verified `dg5f_v1_joint26_bootstrap`, global step zero
- Final checkpoint SHA-256:
  `e35060b5776531ec664715658a8fdb5ef79b98fc23442745d900bb9fd5078807`
- Local result directory:
  `training/results/dg5f_v2_joint26_gpu_fixed`
- Local archive:
  `training/archives/dg5f_v2_joint26_lr1e4_failed_253740.tar.gz`
- Local archive SHA-256:
  `fd4eb2df71901c2729f582433a5d7f0401a29a7f0ccf94f8748bca197f964a4b`

The run was stopped cleanly and exported at step 253740. Non-finite physics
remained zero, and reach improved, but the pilot did not satisfy the required
`Reach/Success >= 80%` gate. In the trend windows used at the stop decision,
reach rose from about 39.9% (0..100k) to 56.4% (100k..200k), while grasp
success fell from about 52.7% to 43.5%, dual contact fell from about 69.8% to
46.1%, and timeout rose from about 9.2% to 41.0%. The most recent 20k window
was worse for hand learning: grasp about 36.2%, dual contact about 37.5%, and
timeout about 55.6%.

This is a failed comparison run. It must not be resumed or selected as the V3
transfer source. The subsequent `dg5f_v2_joint26_lr5e5_gpu_fixed` run also
failed and is documented separately. The active replacement is
`dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed`, initialized afresh from the same
frozen bootstrap at step zero with learning rate `5e-5`.
