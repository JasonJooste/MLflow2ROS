#!/usr/bin/env bash
set -e

# The path to this script
SCRIPTPATH="$(dirname "$0")"
source "$SCRIPTPATH/config.cfg"

docker run -it --network host $DOCKER_IMAGE