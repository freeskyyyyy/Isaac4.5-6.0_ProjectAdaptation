#!/usr/bin/env bash
# Sdf-only audit tools need the USD libraries without launching Kit or a GPU.
set -euo pipefail
sim_root="${ISAAC_SIM_ROOT:-/isaac-sim}"
usd_libs=$(find "$sim_root/extscache" -maxdepth 1 -type d -name 'omni.usd.libs-*' -print -quit)
if [[ -z "$usd_libs" ]]; then
    echo 'Cannot locate the bundled USD libraries' >&2
    exit 1
fi
export PYTHONPATH="$usd_libs:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$usd_libs/bin:$sim_root/kit:$sim_root/kit/plugins:${LD_LIBRARY_PATH:-}"
exec "$sim_root/python.sh" "$@"
