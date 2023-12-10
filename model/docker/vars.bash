#!/usr/bin/env bash
set -o errexit

# A script to store and calculate the variables that are necessary for building
# and running the docker image.
# Environment vars that will not be overwritten:
#   $DOCKER_ARGS: array containing additional Docker arguments (will be split on " ")
#   $DOCKER_NETWORK: value of `docker run`'s `--network` argument (default is "host")
#   $NVIDIA: set to "false" to not try to use nvidia-docker2

# The path to this script
SCRIPTPATH="$(dirname "$BASH_SOURCE")"
# The workspace for editing files in a container
WORKSPACEPATH="$(realpath "$SCRIPTPATH/..")"

# Default container name
if [ -z "$DOCKER_NAME" ]; then
  DOCKER_NAME="placog-egonn"
fi

# Default image
if [ -z "$DOCKER_TAG" ]; then
  DOCKER_TAG="$(cd "$SCRIPTPATH"; git rev-parse --abbrev-ref HEAD | sed -E s/[^a-zA-Z0-9_\.\-]/_/g || echo latest)"
fi
if [ -z "$DOCKER_IMAGE" ]; then
  DOCKER_IMAGE="docker-registry.it.csiro.au/refarm/placog-egonn:$DOCKER_TAG"
fi

# Default network
if [ -z "$DOCKER_NETWORK" ]; then
  DOCKER_NETWORK="host"
fi

# Determine if gpus should be added - default is true if it is possible
if [ "$NVIDIA" != "false" ]; then
  NVIDIA="true"
fi
if [ "$NVIDIA" == "true" ]; then
  # Use lspci to check for the presence of a NVIDIA graphics card
  HAS_NVIDIA="$(lspci | grep -i nvidia | wc -l)"
  # Check if nvidia-container-toolkit or nvidia-docker2 is installed
  HAS_NVIDIA_DOCKER2="$(dpkg -l | grep nvidia-docker2 | wc -l)"
  HAS_NVIDIA_CONTAINER_TOOLKIT="$(dpkg -l | grep nvidia-container-toolkit | wc -l)"
  HAS_NVIDIA_CONTAINER_SUPPORT="$((${HAS_NVIDIA_DOCKER2} + ${HAS_NVIDIA_CONTAINER_TOOLKIT}))"
fi
