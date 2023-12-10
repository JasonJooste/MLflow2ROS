#!/usr/bin/env bash
set -e

# Build a Docker image stage
# usage: build_stage target_stage opts
SCRIPTPATH="$(dirname "$0")"
TARGET_STAGE="$1"
OPTS="${@:2}"
# Get the image info
. "$SCRIPTPATH/vars.bash"

docker build \
  --tag "$DOCKER_IMAGE" \
  --file "$SCRIPTPATH/Dockerfile" \
  --target "$TARGET_STAGE" \
  $OPTS \
  "$SCRIPTPATH/.."
