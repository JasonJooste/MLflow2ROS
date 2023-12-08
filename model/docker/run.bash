#!/usr/bin/env bash

set -e

# Run the Docker container optionally with nvidia-docker2 support
# All arguments to this script will be appended to a docker run command.
# Environment vars that will be read:
#   $DOCKER_ARGS: array containing additional Docker arguments (will be split on " ")
#   $DOCKER_IMAGE: Docker image to run
#   $DOCKER_NAME: value of `docker run`'s `--name` argument
#   $DOCKER_NETWORK: value of `docker run`'s `--network` argument (default is "host")
#   $NVIDIA: set to "false" to not try to use nvidia-docker2

# Example command line:
# ./run.bash /bin/bash

# Get the path to this script
SCRIPTPATH="$(dirname "$0")"

# Read environment vars
IFS=" " read -a DOCKER_ARGS <<< "$DOCKER_ARGS"

# Container image
if [ -z "$DOCKER_IMAGE" ]; then
  DOCKER_IMAGE="logreg-test-modeli"
fi

# Container name
if [ -z "$DOCKER_NAME" ]; then
  DOCKER_NAME="logreg-test-modelc"
fi
DOCKER_ARGS+=("--name")
DOCKER_ARGS+=("$DOCKER_NAME")

# Container network
if [ -z "$DOCKER_NETWORK" ]; then
  DOCKER_NETWORK="host"
fi
DOCKER_ARGS+=("--network")
DOCKER_ARGS+=("$DOCKER_NETWORK")

# Start interactive if stdin is a terminal
if [ -t 0 ]; then
  DOCKER_ARGS+=("--interactive")
  DOCKER_ARGS+=("--tty")
fi

# ROS
if [ -n "$ROS_MASTER_URI" ]; then
  DOCKER_ARGS+=("--env")
  DOCKER_ARGS+=("ROS_MASTER_URI")
fi

# Start container
echo "$DOCKER_IMAGE"
echo "${DOCKER_ARGS[@]}"
docker run \
  --privileged \
  --rm \
  "${DOCKER_ARGS[@]}" \
  "$DOCKER_IMAGE" \
  "$@"
