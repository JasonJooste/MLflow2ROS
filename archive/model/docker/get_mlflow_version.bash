#!/usr/bin/env bash
# Use a mlflow docker image to access the version number of the production version of a registered model
MODEL="tracking-quickstart"
FILTER_STR="name=\'$MODEL\'"
STAGE=Production
VERSION_CMD="import mlflow;v=[m for m in mlflow.search_model_versions(filter_string=\"$FILTER_STR\") if m.current_stage==\"$STAGE\"][0].version;print(v)"
MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"
MODEL_VERSION=$(docker run -e MLFLOW_TRACKING_URI=$MLFLOW_TRACKING_URI ghcr.io/mlflow/mlflow /usr/local/bin/python -c "$VERSION_CMD")
echo $MODEL_VERSION
