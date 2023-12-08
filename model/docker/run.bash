#!/usr/bin/env bash
set -o errexit

# Run the Docker container optionally with nvidia-docker2 support
# All arguments to this script will be appended to a docker run command.
# See vars.sh for env var handling

# Example command line:
# ./run.bash /bin/bash

# Get the path to this script
SCRIPTPATH="$(dirname "$0")"
# Source the vars
. "${SCRIPTPATH}/vars.bash"

# Read in the docker args from a string
read -a DOCKER_ARGS <<< "${DOCKER_ARGS}"

DOCKER_ARGS+=("--name")
DOCKER_ARGS+=("${DOCKER_NAME}")

DOCKER_ARGS+=("--network")
DOCKER_ARGS+=("${DOCKER_NETWORK}")

# Start interactive if stdin is a terminal
if [ -t 0 ]; then
  DOCKER_ARGS+=("--interactive")
  DOCKER_ARGS+=("--tty")
fi

# If both check out, use nvidia-docker2
if [ "${NVIDIA}" == "true" ] && [ "${HAS_NVIDIA}" -gt 0 ] && [ "${HAS_NVIDIA_CONTAINER_SUPPORT}" -gt 0 ]; then
  DOCKER_ARGS+=("--env"); DOCKER_ARGS+=("NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics")
  DOCKER_ARGS+=("--env"); DOCKER_ARGS+=("NVIDIA_VISIBLE_DEVICES=all")
  DOCKER_ARGS+=("--gpus"); DOCKER_ARGS+=("all")
  DOCKER_ARGS+=("--runtime"); DOCKER_ARGS+=("nvidia")
fi

# ROS
if [ -n "${ROS_MASTER_URI}" ]; then
  DOCKER_ARGS+=("--env")
  DOCKER_ARGS+=("ROS_MASTER_URI")
fi

# Start container
set -o xtrace
docker run \
  --privileged \
  --rm \
  "${DOCKER_ARGS[@]}" \
  "${DOCKER_IMAGE}" \
  "$@"
