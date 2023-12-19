#!/usr/bin/env bash
set -e

SCRIPTPATH="$(dirname "$0")"
MODEL_URI="models:/tracking-quickstart/1"
DOCKER_IMAGE="testi"
export ROS_DISTRO="noetic"
UBUNTU_DISTRO="focal"

# TODO: is this required here?
export MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"

mlflow models generate-dockerfile --model-uri "$MODEL_URI" --output-directory "$SCRIPTPATH/dockerfile"

# Add `as base` to the initial build stage
sed -i '/FROM ubuntu/ s/$/ as base/' $SCRIPTPATH/dockerfile/Dockerfile

# Remove the entrypoint command in the dockerfile as we will define a new one later
sed -i '/ENTRYPOINT/d' $SCRIPTPATH/dockerfile/Dockerfile

cat $SCRIPTPATH/Install.Dockerfile >> $SCRIPTPATH/dockerfile/Dockerfile

cp -r ./src ./dockerfile

docker build --tag "$DOCKER_IMAGE" $SCRIPTPATH/dockerfile