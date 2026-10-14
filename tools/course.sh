#!/usr/bin/env bash
# Run on the Spark host; every simulator/ROS/vision command executes in one container.
set -euo pipefail
course_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$course_root"
export COURSE_ARCH=arm64
export COURSE_HOST_GID=$(id -g)
runtime="$course_root/.runtime/arm64"
container=bobac-isaac6-course

prepare() {
    [[ $(uname -m) == aarch64 ]] || { echo 'This deployment targets DGX Spark ARM64.' >&2; exit 1; }
    mkdir -p "$runtime"/{build,install,log,isaac-cache,compute-cache,isaac-config,isaac-logs,vision}
    [[ -f "${ISAAC_SIM_HOST_DIR:-$course_root/../IsaacSim-6.0.0}/VERSION" ]] || {
        echo 'Desktop Isaac Sim installation is missing; run bash tools/setup_desktop_sdk.sh first.' >&2
        exit 1
    }
    mkdir -p reports scenes
    chmod g+rwXs reports scenes
    if [[ -z ${DISPLAY:-} ]]; then
        if [[ -S /tmp/.X11-unix/X1 ]]; then export DISPLAY=:1;
        elif [[ -S /tmp/.X11-unix/X0 ]]; then export DISPLAY=:0; fi
    fi
    local auth="${XAUTHORITY:-}"
    if [[ -z "$auth" ]]; then
        for candidate in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
            if [[ -r "$candidate" ]]; then auth="$candidate"; break; fi
        done
    fi
    local empty_auth=''
    if [[ -z "$auth" || ! -r "$auth" ]]; then
        empty_auth=$(mktemp /tmp/bobac-course-auth.XXXXXX)
        auth="$empty_auth"
    fi
    # Only task-owned runtime directories are changed; source files retain their ownership.
    docker run --rm --user 0:0 --entrypoint bash \
      -v "$runtime:/runtime" \
      --mount "type=bind,source=$auth,target=/tmp/source-auth,readonly" \
      bobac-course:isaac6.0-jazzy-spark \
      -c 'cat /tmp/source-auth > /runtime/Xauthority; chown -R 1234:1234 /runtime; chmod 600 /runtime/Xauthority'
    if [[ -n "$empty_auth" ]]; then rm -- "$empty_auth"; fi
}

inside() {
    docker exec -it "$container" /opt/course/entrypoint.sh "$@"
}

case "${1:-help}" in
    build)
        docker compose build
        ;;
    up)
        prepare
        docker compose up -d
        # Copy the image's verified install into its persistent workspace once.
        if ! docker exec "$container" test -f /workspace/bobac_ws/install/setup.bash; then
            docker exec --user 0:0 "$container" bash -c \
                'cp -a /opt/course/prebuilt-install/. /workspace/bobac_ws/install/; chown -R 1234:1234 /workspace/bobac_ws/install'
        fi
        ;;
    check)
        inside bash -c 'cd /isaac-sim; ./isaac-sim.compatibility_check.sh --/app/quitAfter=10 --no-window' \
          | tee reports/compatibility-check.log
        ;;
    compile)
        inside bash -c 'cd "$COURSE_WS"; colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
          | tee reports/container-jazzy-build.log
        ;;
    audit)
        inside /opt/course/usd-python.sh /workspace/course/tools/scene_audit.py \
          /workspace/course/test_scene_bobac.usd --report /workspace/course/reports/scene-audit.json \
          --draft /workspace/course/scenes/test_scene_bobac_6.0.draft.usda \
          --asset-root /workspace/course/isaacsimassets-dev/assets --solid-markings
        ;;
    migrate)
        inside /isaac-sim/python.sh /workspace/course/tools/migrate_scene.py \
          /workspace/course/scenes/test_scene_bobac_6.0.draft.usda \
          --output /workspace/course/scenes/test_scene_bobac_6.0.usda \
          --report /workspace/course/reports/scene-migration.json
        ;;
    sim)
        inside /isaac-sim/python.sh /workspace/course/tools/run_sim.py \
          --scene /workspace/course/scenes/test_scene_bobac_6.0.usda
        ;;
    ros)
        inside ros2 launch course_bringup course.launch.py
        ;;
    ready)
        inside ros2 run course_bringup course_readiness --timeout 60 \
          --report /workspace/course/reports/ros-readiness.json
        ;;
    vision-check)
        inside /opt/course-vision/bin/python /workspace/course/tools/vision_check.py
        ;;
    grasp)
        inside python3 /workspace/course/tools/action_readiness.py
        inside ros2 run grasp_control grasp_state_machine --ros-args -p use_sim_time:=true
        ;;
    place)
        inside python3 /workspace/course/tools/action_readiness.py
        inside ros2 run grasp_control place_state_machine --ros-args -p use_sim_time:=true
        ;;
    locks)
        docker cp "$container:/opt/course/vision-resolved.txt" reports/vision-resolved.txt
        docker cp "$container:/opt/course/apt-resolved.txt" reports/apt-resolved.txt
        docker image inspect bobac-course:isaac6.0-jazzy-spark > reports/course-image.json
        ;;
    shell)
        inside bash
        ;;
    down)
        docker compose down
        ;;
    *)
        echo 'Usage: bash tools/course.sh {build|up|check|compile|audit|migrate|sim|ros|ready|vision-check|grasp|place|locks|shell|down}'
        ;;
esac
