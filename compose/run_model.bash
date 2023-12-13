#!/usr/bin/env bash

docker run -p 5002:8080 logreg_mlops

# curl -d '{"inputs": [[6.1, 2.8, 4.7, 1.2],  [5.7, 3.8, 1.7, 0.3]]}' -H 'Content-Type: application/json' -X POST localhost:5002/invocations
