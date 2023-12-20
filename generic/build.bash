#!/usr/bin/env bash
set -e

MODEL_URI="models:/tracking-quickstart/1"
DOCKER_IMAGE="testi"

SCRIPTPATH="$(dirname "$0")"
DOCFILE=$SCRIPTPATH/dockerfile/Dockerfile

export MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"

mlflow models generate-dockerfile --model-uri "$MODEL_URI" --output-directory "$SCRIPTPATH/dockerfile"

# Add `as base` to the initial build stage
sed -i '/FROM ubuntu/ s/$/ as base/' $DOCFILE

# Remove the entrypoint command in the dockerfile as we will define a new one later
sed -i '/ENTRYPOINT/d' $DOCFILE

# Edit the COPY command to work with our build context
sed -i '/model_dir/c\COPY dockerfile/model_dir /opt/ml/model' $DOCFILE

cat $SCRIPTPATH/Install.Dockerfile >> $DOCFILE

docker build --tag "$DOCKER_IMAGE" --file $DOCFILE $SCRIPTPATH