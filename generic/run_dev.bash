#!/usr/bin/env bash
set -e

# The path to this script
SCRIPTPATH="$(dirname "$0")"
source "$SCRIPTPATH/config.cfg"

# The workspace for editing files in a container
WORKSPACEPATH="$(realpath "$SCRIPTPATH/..")"

docker run -it --network host --mount type=bind,source=$WORKSPACEPATH,destination=/workspace/ $DOCKER_IMAGE