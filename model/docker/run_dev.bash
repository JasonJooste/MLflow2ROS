#!/usr/bin/env bash
set -e

# Run a command with your local workspace mounted in the Docker container
SCRIPTPATH="$(dirname "$0")"
# Source local vars
. "$SCRIPTPATH/vars.bash" 
# Add mount for workspace
export DOCKER_ARGS+=" --mount type=bind,source=$WORKSPACEPATH,destination=/workspace/"
"$SCRIPTPATH/run.bash" "$@"
