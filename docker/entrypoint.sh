#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ "${SOURCE_WORKSPACE:-0}" = "1" ] && [ -f /workspaces/DOBOT_pickn_place/install/setup.bash ]; then
  source /workspaces/DOBOT_pickn_place/install/setup.bash
fi

exec "$@"
