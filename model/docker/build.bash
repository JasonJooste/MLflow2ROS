#!/usr/bin/env bash
set -e
SCRIPTPATH="$(dirname "$0")"
docker build --force-rm --tag "logreg-test-modeli" --file "$SCRIPTPATH/Dockerfile" "$SCRIPTPATH/.."
