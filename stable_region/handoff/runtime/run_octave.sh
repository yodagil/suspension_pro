#!/usr/bin/env bash
set -euo pipefail
HMC_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HMC_PROJECT="${HMC_PROJECT_ROOT:-$HMC_SCRIPT_DIR}"
while [[ ! -f "$HMC_PROJECT/HMC_FullCar_Revised.m" && "$HMC_PROJECT" != / ]]; do
  HMC_PROJECT="$(dirname -- "$HMC_PROJECT")"
done
[[ -f "$HMC_PROJECT/HMC_FullCar_Revised.m" ]] || { echo 'Set HMC_PROJECT_ROOT to the extracted project directory.' >&2; exit 2; }
HMC_RUNTIME="${HMC_RUNTIME_ROOT:-$HMC_PROJECT/.runtime}"
HMC_PREFIX="${HMC_OCTAVE_PREFIX:-$HMC_RUNTIME/env}"
HMC_MAMBA="$HMC_RUNTIME/bin/micromamba"
[[ -x "$HMC_MAMBA" && -x "$HMC_PREFIX/bin/octave-cli" ]] || {
  echo 'Run setup_octave.sh first, or set HMC_OCTAVE_PREFIX to the matching existing environment.' >&2; exit 2;
}
mkdir -p "$HMC_RUNTIME/tmp" "$HMC_RUNTIME/app-cache" "$HMC_RUNTIME/app-config" "$HMC_RUNTIME/app-data" "$HMC_RUNTIME/cache"
export TMPDIR="$HMC_RUNTIME/tmp"
export XDG_CACHE_HOME="$HMC_RUNTIME/app-cache"
export XDG_CONFIG_HOME="$HMC_RUNTIME/app-config"
export XDG_DATA_HOME="$HMC_RUNTIME/app-data"
export OCTAVE_HISTFILE="$HMC_RUNTIME/app-data/octave-history"
# Preserve the original MEX bytes/RPATH. Supply relocated exact runtime libraries.
export LD_LIBRARY_PATH="$HMC_PREFIX/lib:$HMC_PREFIX/lib/octave/10.3.0${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$HMC_MAMBA" --no-rc --root-prefix "$HMC_RUNTIME/cache" run --prefix "$HMC_PREFIX" "$@"
