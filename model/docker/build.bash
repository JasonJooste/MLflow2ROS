#!/usr/bin/env bash
set -e

SCRIPTPATH="$(dirname "$0")"
# Pull in the evironment variables
. "$SCRIPTPATH/vars.bash"
# Build all stages of the Docker image
"$SCRIPTPATH/build_stage.bash" frozen_stage
"$SCRIPTPATH/run_deps.bash"
"$SCRIPTPATH/build_stage.bash" base_stage
"$SCRIPTPATH/run_compile.bash"
# MLFlow model version
MODEL_VERSION=$("$SCRIPTPATH/get_mlflow_version.bash") 
"$SCRIPTPATH/build_stage.bash" prod_stage --build-arg MODEL_VERSION=$MODEL_VERSION

