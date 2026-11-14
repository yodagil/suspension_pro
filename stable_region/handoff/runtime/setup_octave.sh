#!/usr/bin/env bash
set -euo pipefail
HMC_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HMC_PROJECT="${HMC_PROJECT_ROOT:-$HMC_SCRIPT_DIR}"
while [[ ! -f "$HMC_PROJECT/HMC_FullCar_Revised.m" && "$HMC_PROJECT" != / ]]; do HMC_PROJECT="$(dirname -- "$HMC_PROJECT")"; done
[[ -f "$HMC_PROJECT/HMC_FullCar_Revised.m" ]] || { echo 'Set HMC_PROJECT_ROOT to the extracted project directory.' >&2; exit 2; }
HMC_RUNTIME="${HMC_RUNTIME_ROOT:-$HMC_PROJECT/.runtime}"
HMC_PREFIX="${HMC_OCTAVE_PREFIX:-$HMC_RUNTIME/env}"
if [[ "${1:-}" == --prefix ]]; then
  [[ $# == 2 ]] || { echo 'Usage: setup_octave.sh [--prefix EXISTING_OR_NEW_PREFIX]' >&2; exit 2; }
  HMC_PREFIX="$2"
elif [[ $# != 0 ]]; then echo 'Usage: setup_octave.sh [--prefix PREFIX]' >&2; exit 2; fi
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || { echo 'The frozen runtime/MEX targets Linux x86_64, not Windows/macOS/ARM.' >&2; exit 2; }
python3 "$HMC_SCRIPT_DIR/verify_runtime_inputs.py" "$HMC_SCRIPT_DIR"
# The exact conda lock includes x86-64-v4/Ice Lake packages.
python3 - <<'PY'
from pathlib import Path
flags=set()
for line in Path('/proc/cpuinfo').read_text().splitlines():
    if line.startswith('flags'):
        flags=set(line.split(':',1)[1].split());break
need={'avx2','fma','avx512f','avx512bw','avx512cd','avx512dq','avx512vl'}
missing=sorted(need-flags)
if missing: raise SystemExit('Exact runtime needs an x86-64-v4/AVX-512 host; missing: '+','.join(missing)+'. Use a compatible Linux host; do not silently rebuild/replace the frozen binaries.')
PY
mkdir -p "$HMC_RUNTIME/bin" "$HMC_RUNTIME/downloads" "$HMC_RUNTIME/tmp" "$HMC_RUNTIME/cache/pkgs"
if [[ ! -x "$HMC_RUNTIME/bin/micromamba" ]]; then
  tar -xjf "$HMC_SCRIPT_DIR/micromamba_linux64.tar.bz2" -C "$HMC_RUNTIME" bin/micromamba
fi
if [[ ! -x "$HMC_PREFIX/bin/octave-cli" ]]; then
  # micromamba create attempts to register an environment in HOME. The
  # original managed workspace has read-only HOME, so installation uses the
  # locked conda-standalone with environment registration explicitly disabled.
  HMC_CONDA_ARCHIVE="$HMC_RUNTIME/downloads/conda-standalone-26.9.4.post1-h643be8f_0.conda"
  if [[ ! -f "$HMC_CONDA_ARCHIVE" ]]; then
    curl --fail --location --retry 3 \
      'https://conda.anaconda.org/conda-forge/linux-64/conda-standalone-26.9.4.post1-h643be8f_0.conda' \
      --output "$HMC_CONDA_ARCHIVE.tmp"
    mv "$HMC_CONDA_ARCHIVE.tmp" "$HMC_CONDA_ARCHIVE"
  fi
  python3 - "$HMC_CONDA_ARCHIVE" <<'PY'
import hashlib,sys
from pathlib import Path
p=Path(sys.argv[1]);wanted='16ca154943dee985b84294c20baa81bce1e935eb0896be51e2d15ea5a45ac66b'
if hashlib.sha256(p.read_bytes()).hexdigest()!=wanted: raise SystemExit('conda-standalone archive SHA-256 mismatch; installation stopped.')
PY
  if ! python3 -c 'import zstandard' >/dev/null 2>&1; then
    python3 -m pip install --target "$HMC_RUNTIME/bootstrap-deps" 'zstandard==0.25.0'
  fi
  PYTHONPATH="$HMC_RUNTIME/bootstrap-deps${PYTHONPATH:+:$PYTHONPATH}" python3 "$HMC_SCRIPT_DIR/extract_conda_standalone.py" "$HMC_CONDA_ARCHIVE" "$HMC_RUNTIME/bin/conda-standalone"
  HMC_CONDARC="$HMC_RUNTIME/condarc"
  python3 - "$HMC_RUNTIME" "$HMC_CONDARC" <<'PY'
import json,sys
from pathlib import Path
r=Path(sys.argv[1]);Path(sys.argv[2]).write_text('register_envs: false\nnotify_outdated_conda: false\nchannels:\n  - conda-forge\nchannel_priority: strict\nenvs_dirs:\n  - '+json.dumps(str(r/'cache/envs'))+'\npkgs_dirs:\n  - '+json.dumps(str(r/'cache/pkgs'))+'\n')
PY
  CONDARC="$HMC_CONDARC" CONDA_PKGS_DIRS="$HMC_RUNTIME/cache/pkgs" \
    "$HMC_RUNTIME/bin/conda-standalone" create --yes --prefix "$HMC_PREFIX" \
    --file "$HMC_SCRIPT_DIR/environment-explicit.txt"
fi
export HMC_PROJECT_ROOT="$HMC_PROJECT" HMC_RUNTIME_ROOT="$HMC_RUNTIME" HMC_OCTAVE_PREFIX="$HMC_PREFIX"
bash "$HMC_SCRIPT_DIR/run_octave.sh" octave-cli --quiet --no-init-file --no-site-file --no-history \
  --eval "assert(strcmp(OCTAVE_VERSION,'10.3.0')); fprintf('GNU Octave %s runtime ready\\n',OCTAVE_VERSION);"
printf 'Use HMC_OCTAVE_PREFIX=%q when invoking run_octave.sh if this differs from its default.\n' "$HMC_PREFIX"
