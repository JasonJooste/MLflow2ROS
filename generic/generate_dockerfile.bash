#!/usr/bin/env bash
set -e

SCRIPTPATH="$(dirname "$0")"
MODEL_URI="models:/tracking-quickstart/1"
export MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"

mlflow models generate-dockerfile --model-uri "$MODEL_URI" --output-directory "$SCRIPTPATH/dockerfile"

# Remove the entrypoint command in the dockerfile
sed -i '/ENTRYPOINT/d' $SCRIPTPATH/dockerfile/Dockerfile