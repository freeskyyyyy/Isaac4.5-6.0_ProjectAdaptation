#!/usr/bin/env bash
set -e
umask 0002
source /opt/ros/jazzy/setup.bash
if [[ -f "${COURSE_WS:-/workspace/bobac_ws}/install/setup.bash" ]]; then
    source "${COURSE_WS:-/workspace/bobac_ws}/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
exec "$@"
