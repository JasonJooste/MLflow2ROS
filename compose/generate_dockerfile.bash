#!/usr/bin/env bash

export MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"

mlflow models generate-dockerfile --model-uri "models:/tracking-quickstart/1"