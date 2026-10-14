#!/usr/bin/env bash
# Export the official SDK to the desktop before binding it over /isaac-sim.
set -euo pipefail
course_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
sdk_dir="${ISAAC_SIM_HOST_DIR:-$course_root/../IsaacSim-6.0.0}"
image='nvcr.io/nvidia/isaac-sim:6.0.0@sha256:68735a60b6c15c85e0dd0098570c6d2cc79e928f2d068ce2790aa43284ac165d'
if [[ ! -e "$sdk_dir" ]]; then
    sdk_container=$(docker create --entrypoint /bin/true "$image")
    trap 'docker rm "$sdk_container" >/dev/null' EXIT
    docker cp "$sdk_container:/isaac-sim" "$sdk_dir"
fi
[[ -f "$sdk_dir/VERSION" && -f "$sdk_dir/python.sh" && -d "$sdk_dir/kit" ]] || {
    echo "Incomplete desktop SDK directory: $sdk_dir" >&2; exit 1;
}
[[ $(cat "$sdk_dir/VERSION") == 6.0.0* ]] || {
    echo 'Desktop SDK is not Isaac Sim 6.0.0; select a separate directory.' >&2; exit 1;
}
docker run --rm --user 0:0 --entrypoint bash -v "$sdk_dir:/desktop-sdk" \
    -e "COURSE_HOST_GID=$(id -g)" "$image" \
    -c 'chown -R 1234:"$COURSE_HOST_GID" /desktop-sdk; chmod -R a+rX,g+w /desktop-sdk'
printf '%s\n' "$image" > "$sdk_dir/.course-image-pin"
printf 'Desktop SDK ready: %s -> /isaac-sim\n' "$sdk_dir"
