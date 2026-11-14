# GNU Octave compatibility

The physical model and its C++ evaluator are the files from the supplied archive.
The adapter in this folder supplies CSV table export for GNU Octave and Tablicious.
Add this folder only in an Octave session; MATLAB uses its own `writetable`.

Use Tablicious v0.4.7, commit `8a1b10faf994c5566e6f08242af360e017ba2f52`,
from <https://github.com/apjanke/octave-tablicious>. Install it with Octave's
package manager, or set `TABLICIOUS_ROOT` to that source checkout before running
`bash scripts/run_octave.sh`. Tablicious is an external GPLv3+ dependency and is
not bundled in this repository.

The Octave runner compiles `hmc_mex.cpp` with `mkoctfile`, then runs the supplied
six-case MATLAB-source/native implementation equivalence check before using the
accelerator. This compares implementations within Octave; it does not establish
cross-runtime equivalence with a licensed MATLAB execution or CarSim accuracy.

Each result records its actual runtime and version. In Octave, `run_status.mat`
stores `Summary` as a plain structure array so MATLAB can read it without the
Tablicious class. Condition results remain ordinary numeric MAT structures.
