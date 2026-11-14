#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
octave_bin="${OCTAVE_BIN:-octave}"
mkoctfile_bin="${MKOCTFILE_BIN:-mkoctfile}"

if ! command -v "$octave_bin" >/dev/null 2>&1; then
    printf '%s\n' 'GNU Octave was not found. Activate its environment or set OCTAVE_BIN.' >&2
    exit 127
fi
if ! command -v "$mkoctfile_bin" >/dev/null 2>&1; then
    printf '%s\n' 'mkoctfile is required to build the supplied C++ MEX accelerator.' >&2
    exit 127
fi

export HMC_PROJECT_ROOT="$repo_dir"
export HMC_TABLICIOUS_ROOT="${TABLICIOUS_ROOT:-}"
"$mkoctfile_bin" --mex -O2 -std=c++17 -o "$repo_dir/hmc_mex.mex" "$repo_dir/hmc_mex.cpp"
mkdir -p "$repo_dir/result"
"$octave_bin" --no-gui --quiet --no-init-file --no-site-file --no-history --eval \
    "addpath(getenv('HMC_PROJECT_ROOT')); addpath(fullfile(getenv('HMC_PROJECT_ROOT'),'compat','octave')); if isempty(getenv('HMC_TABLICIOUS_ROOT')), pkg('load','tablicious'); else, addpath(fullfile(getenv('HMC_TABLICIOUS_ROOT'),'inst')); end; run_suspension_pro;" \
    2>&1 | tee "$repo_dir/result/execution.log"
