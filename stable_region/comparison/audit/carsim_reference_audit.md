# CarSim region reference audit

All three uploaded files identify CarSim 2016.2 + Simulink, 70 km/h, road mu 0.6, and wear [25 25 0 0]. Original files were opened read-only. Numerical comparison-core MAT exports are in `core/`; the per-point raw metric structs are omitted explicitly in `extraction_scope`. Every grid, H diagnostic, point mask, cell mask, centre point/result, trim, status and wear value in the core export was verified by MAT re-read against the original HDF5 arrays.

| Target steer (deg) | Recorded actual steer | H grid points | final 5/10 s | guarded 5/10 s | recoverable 5/10 s | accepted cells | cell area |
| ---: | ---: | ---: | --- | --- | --- | ---: | ---: |
| 0 | -0.0072287707649889045 | 522 | 401 / 401 | 325 / 325 | 517 / 522 | 343 | 0.8575 |
| 1.5 | 1.3076631615654044 | 517 | 386 / 386 | 300 / 300 | 490 / 516 | 330 | 0.8250 |
| 2.5 | 2.2059404396530846 | 512 | 362 / 362 | 161 / 161 | 452 / 478 | 307 | 0.7675 |

The domain is 61 x 121: initial lateral-velocity command -3:0.1:3 m/s and yaw-rate command -1.5:0.025:1.5 rad/s. Cell area is 0.0025 (m/s)*(rad/s). In all files status 1 exactly equals H-inside and status -2 exactly equals H-outside. There are no incomplete or failed H-inside points. Every saved candidate cell centre passed. H-outside points were not simulated and must not be counted as known unstable points.

## Classification semantics

The files explicitly state observed-output recovery rather than the attached CDC code's 29-state and RHS recovery. The seven terminal tolerances are lateral velocity 0.1 m/s, yaw rate 0.005 rad/s, roll/pitch 0.2 deg, body roll/pitch rate 0.5 deg/s and longitudinal velocity 0.5 km/h. Horizon is 5 and 10 seconds; terminal window is one second. Full trajectories and actual per-point time vectors are not stored.

The connected repository `yodagil/suspension` at commit `eed311e4edfe3d70228363465bd96217ff120adb` contains a later matching port in `hmc_standalone/hmc_classify.m`. This port explicitly uses an inclusive last window `dt>=horizon-window-1e-8`. It applies physical finite/output/load/LTR/roll/positive-speed checks to primary recovery; yaw and lateral-acceleration policies are added only to guarded_final. There is no RHS derivative convergence test. This source is a later policy port; the original `CarSim_1050_Run/cs_classify` generator was not found in the accessible main tree.

## Direct raw metric consistency

All saved raw per-point pass/model-safe/recoverable/guarded fields agree exactly with the corresponding saved masks. Across every H point and both horizons, main pass equals recoverable AND no recorded H exit before or at the horizon. No finite physical-failure time is recorded in any file. Recorded H exits fall on a 1 ms lattice to <=8.81e-16 s, but the files cannot independently establish actual per-point sample count 10001.

## Common H and input matching

The supplied H is labelled an original four-tire MATLAB MF 2x2-Jacobian reference, not a CarSim or full HMC tire Jacobian. A matching port `hmc_standalone/hmc_handling_reference.m` uses the recorded mean front-wheel steering angle for this external reference. For fair first comparison, preserve this H for both masks while using nominal 0, 1.5 or 2.5 deg as actual model commands. The result model dynamics must remain the existing 34-state HMC code.

Initial-condition handshake tolerances are 0.08 m/s and 0.025 rad/s, with up to 0.005 s delay. Raw files retain initial roll rate but not complete actual initial lateral/yaw velocities. Therefore equality of actual initial physical states and the exact observer sampling protocol cannot be independently confirmed. Compare common nominal grids and each model's own trim, without fitted shifts or reference-state injection.
