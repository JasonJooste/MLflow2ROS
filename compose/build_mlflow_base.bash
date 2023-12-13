#!/usr/bin/env bash

export MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"

mlflow models build-docker --model-uri "models:/tracking-quickstart/1" --name "logreg_mlops"