# Independent three-model region audit

All three requested conditions were verified: speed 70 km/h, friction 0.6, wear [25 25 0 0], commanded steering 0°, 1.5°, and 2.5°. Each cell mask is 60×120 on the increasing 61×121 (vy,r) axes. The coordinate spacing is 0.1 m/s and 0.025 rad/s. Every accepted cell has four accepted 10 s grid vertices; saved areas and counts match the masks. No regions were transformed.

| Steering | carsim cells | carsim_fullcar cells | simul_fullcar cells | CarSim/HMC IoU | CarSim/Simul IoU | HMC/Simul IoU |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0° | 343 | 342 | 355 | 0.9970845481 | 0.7363184080 | 0.7338308458 |
| 1.5° | 330 | 330 | 351 | 1.0000000000 | 0.7196969697 | 0.7196969697 |
| 2.5° | 307 | 307 | 318 | 0.9935064935 | 0.7123287671 | 0.7170329670 |

All eight original physics/data/calibration fingerprints match the original result/1050-run signature. The independent audit only read saved results; it ran no dynamics and edited no source inputs.

At 0°, HMC omits one CarSim cell. At 1.5°, both full cell masks and 10 s point masks are identical. At 2.5°, both have 307 accepted cells, but two cells differ: one only in each model. The matching total area therefore does not establish identical regions.

The JSON includes every differing-cell coordinate bound for every model pair, source-file fingerprints, and point agreement restricted to the intersection of the tested H masks. Points outside H are untested rather than valid true negatives.

Caveats:
- Finite-time sampled recovery and reference-H preservation are not an invariant-region certificate.
- carsim and fresh carsim_fullcar use the same external H reconstructed at the recorded CarSim actual steering angle; simul_fullcar H matches the nominal commanded angle.
- CarSim uses seven observed return channels. The LUT describes a 32-state full-car model but does not preserve its complete terminal tolerances or raw trajectories; a strict same-criterion three-way dynamics ranking is not established.
- Original CarSim achieved initial-condition frame is not recorded; comparison uses corresponding nominal commanded initial-grid coordinates.
- Points outside H are untested; point agreement is evaluated only in the intersection of tested H masks.
