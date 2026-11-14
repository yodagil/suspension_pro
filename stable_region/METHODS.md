# Three-model finite-time stability-region comparison

## Scope and model binding

The selected conditions are speed 70 km/h, friction coefficient 0.6, wear vector [25 25 0 0], and nominal commanded steering 0, 1.5, or 2.5 degrees. Names are exactly `carsim`, `carsim_fullcar`, and `simul_fullcar`.

The fresh `carsim_fullcar` calculations use the original result/1050-run 34-state HMC vehicle, MF6.2 tyres, calibration, speed PI states, sampled 1 ms CDC and 2 ms held-force delay. Model/data/calibration/dependency fingerprints match the original run signature. The supplied 32-state CDC code was not substituted as the vehicle dynamics. Its static reference-H calculation was used only to reconstruct the reference classification function.

The native observer includes the original unmodified `hmc_mex.cpp`, which includes the original dynamics. Original actions are delegated to the original module implementation. The new action streams region classifications while integrating the same model. The production source and binary were frozen before all three accepted calculations; their hashes are in validation/binary_binding.json and each saved result. Prototype output produced during development was discarded and was not used.

## Inputs and coordinates

`carsim` and `simul_fullcar` boundaries come directly from saved final-10-second cell masks in the uploaded files, not reconstructed display hulls. The nominal initial grid has 61 lateral-velocity nodes from -3 to 3 m/s at 0.1 m/s spacing and 121 yaw-rate nodes from -1.5 to 1.5 rad/s at 0.025 rad/s spacing. Cell arrays are 60 by 120 in (vy,r) order. ULP-level differences in grid storage are checked and compared by corresponding indices, without interpolation.

Original CarSim achieved initial-condition coordinate frames were not saved. All figures therefore use corresponding nominal requested initial-grid coordinates. HMC uses its existing lift function and corrected body-y initial velocity request, original hidden trim states, and the original zero tyre restart policy. Regions were not translated or fitted to improve overlap.

## Common CarSim reference H

The external static H reference for `carsim_fullcar` is reconstructed at CarSim's recorded actual front-wheel steering: -0.0072287707649889, 1.3076631615654044, and 2.2059404396530846 degrees, respectively. Its three grid masks exactly match the uploaded CarSim reference masks (522, 517, 512 tested nodes). This reference only classifies outputs; it does not change the HMC steering command or equations.

HMC front-wheel steering at its own trim is approximately 0.00492528, 1.31896, and 2.20914 degrees for those same nominal commands. Each fresh trajectory is tested against its own HMC trim, not the CarSim trim. All three own trims lie inside the common H.

The supplied `simul_fullcar` H is instead consistent with nominal commanded steering. Its H grid differs from CarSim at 1, 54, and 79 nodes. This reference-definition difference is part of the literal saved-region comparison.

## Fresh finite-time classification

RK4 integration uses 0.0005 s steps and observations every 0.001 s. Both 5 s and 10 s results are saved; figures use 10 s. The model's original sampled/delayed CDC timing and RK4 force-hold ordering are retained.

The CarSim-compatible main policy requires the whole observed path to preserve H and the physical-output conditions, and the final closed one-second observation window to return within the following tolerances of the model's own trim:

| Observable | Tolerance |
| --- | ---: |
| lateral velocity | 0.1 m/s |
| yaw rate | 0.005 rad/s |
| roll angle | 0.2 degrees |
| pitch angle | 0.2 degrees |
| roll rate | 0.5 degrees/s |
| pitch rate | 0.5 degrees/s |
| longitudinal speed | 0.5 km/h |

The terminal window includes its starting sample: [horizon - 1 s, horizon]. Observations and physical quantities must be finite, all four raw wheel normal loads positive, global/front/rear absolute LTR below 1, absolute roll below 40 degrees, and longitudinal speed positive. The model's own trim must lie inside H.

The auxiliary yaw-rate and lateral-acceleration policy limits are included only in `guarded_final`, not in the plotted main region. They must not silently remove main-region points. Physical/H violations allow exact early rejection of the main result because its whole-path condition cannot recover; auxiliary recovery diagnostics after such pruning are NaN/unknown.

Only nodes inside the reference H are simulated. Points outside H are **untested**, not demonstrated unstable. Four passing cell vertices nominate a candidate cell; its actual center is then separately integrated and must pass the same 10 s policy. The fresh conditions used 522+342, 517+330, and 512+307 vertex-plus-center trajectories. All nominated centers passed. Cell unions remain sampled finite-time regions; testing vertices and centers does not certify every continuous point inside each cell.

The supplied LUT describes a different 32-state full-car model but does not preserve its full terminal tolerances or raw trajectories. Complete three-way equivalence of the return criterion cannot be established. Its regions are displayed and measured as supplied.

## Metrics and plotting

Boundaries are exposed edges of the exact accepted-cell union, including holes and disconnected pieces. No convex hull, smoothing, shifting, or interpolation is used. The area unit is (m/s)·(rad/s), not square metres.

IoU = intersection area / union area; Dice = 2×intersection area / (area A + area B). Difference counts are symmetric-difference cell counts. Point-mask agreement uses only the intersection of tested H masks. Untested exterior points are never counted as true negatives. The CSV and JSON include all three model pairs.

## Verification and practical limits

GNU Octave 10.3.0 executed the fresh calculations. For three conditions and three probe trajectories each, the new observer was checked against the original native model's full trajectories: RHS, output observations, states, forces, held CDC, and final states matched exactly. Closed terminal-window errors and main/guarded classifications matched the independently archived classifier. Reference masks matched over all 7,381 grid nodes per condition, one-/four-thread masks agreed, and exact early pruning matched full integration. Additional tests cover outside-H trim exclusion, optional guard separation, and the closed-window boundary.

An independent read-only audit checked the final files, metadata, axes, areas, cell counts, point agreement, original source hashes, and every differing-cell coordinate. It confirms IoU 0.9970845481, 1, and 0.9935064935 for CarSim versus carsim_fullcar. Equal total area at 2.5 degrees does not imply identical cell masks.

The figures compare finite-horizon sampled regions under the documented references and policies. They do not establish asymptotic stability, a controlled invariant set, or a pure dynamics-accuracy ranking. This selected three-condition computation does not claim completion of all 1,050 requested stability maps.
