#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"

if [[ -n "${MATLAB_ROOT:-}" ]]; then
    matlab_bin="$MATLAB_ROOT/bin/matlab"
elif command -v matlab >/dev/null 2>&1; then
    matlab_bin="$(command -v matlab)"
else
    printf '%s\n' 'MATLAB was not found. Add matlab to PATH or set MATLAB_ROOT to the installation directory.' >&2
    exit 127
fi
if [[ ! -x "$matlab_bin" ]]; then
    printf '%s\n' 'The selected MATLAB executable is not executable.' >&2
    exit 126
fi

mkdir -p "$repo_dir/result"
"$matlab_bin" -sd "$repo_dir" -batch 'run_suspension_pro' 2>&1 | tee "$repo_dir/result/execution.log"
