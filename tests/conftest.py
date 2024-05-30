import mlflow
import pytest
from pathlib import Path
import os
import subprocess
import socket

@pytest.fixture(scope="session")
def mlflow_server(tmp_path_factory):
    """ Start an in-memory tracking server """
    tmp_path = tmp_path_factory.mktemp("server")
    track_uri = f"file://{tmp_path / 'track'}"
    mlflow.set_tracking_uri(track_uri)
    os.environ["MLFLOW_TRACKING_URI"] = track_uri
    return track_uri
