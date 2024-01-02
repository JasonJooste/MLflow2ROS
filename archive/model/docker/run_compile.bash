#!/usr/bin/env bash
set -e

# Compile the project
SCRIPTPATH="$(dirname "$0")"
"$SCRIPTPATH/run_dev.bash" bash -c "colcon build --event-handlers console_cohesion+ status-"
