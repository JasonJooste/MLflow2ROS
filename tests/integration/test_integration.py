import docker
import pytest

import subprocess
from pathlib import Path

import app
import os
import model_tensor_basic

VALIDATOR_CONTAINER = "integration_test_validator"

@pytest.fixture
def override_tracking_uri(scope="session", autouse=True):
    # save current tracking uri to be restored once tests conclude
    old_tracking_uri = None
    if "MLFLOW_TRACKING_URI" in os.environ:
        old_tracking_uri = os.environ["MLFLOW_TRACKING_URI"]
    os.environ["MLFLOW_TRACKING_URI"] = "http://127.0.0.1:55555"

    yield old_tracking_uri

    if old_tracking_uri is None:
        # remove env var
        del os.environ["MLFLOW_TRACKING_URI"]
    else:
        # restore tracking uri
        os.environ["MLFLOW_TRACKING_URI"] = old_tracking_uri


@pytest.fixture
def docker_client():
    return docker.from_env()


@pytest.fixture
def build_validator_container(scope="session", autouse=True):
    print("Building validator container")

    subprocess.run(["docker", "build", "--tag", VALIDATOR_CONTAINER, Path.cwd()/"tests"/"integration"/"test_nodes"], check=True, text=True)


@pytest.fixture
def roscore_container(docker_client, build_validator_container, scope="session", autouse=True):
    container = docker_client.containers.run(
        VALIDATOR_CONTAINER, "roscore", network="host", detach=True, auto_remove=True)
    yield container
    container.stop()
    

@pytest.fixture
def mlflow_server_container(docker_client, scope="session", autouse=True):
    container = docker_client.containers.run(
        "ghcr.io/mlflow/mlflow:v2.9.2", "mlflow server --host 127.0.0.1 --port 55555",
        network="host", detach=True, auto_remove=True)
    yield container
    container.stop()


def test_tensor_basic(docker_client, mlflow_server_container, roscore_container, build_validator_container, override_tracking_uri):
    test_name = "tensor_basic"
    model_img_name = f"{test_name}_model"

    # log the model
    model_tensor_basic.run_and_log()

    # run app to build image
    # app.make_image(model_name=test_name, model_ver=1, tag=model_img_name)

    print("Starting model container")
    model = docker_client.containers.run(model_img_name, network="host", detach=True, auto_remove=True)

    print("Starting validator container")
    try:
        # must use subprocess since the docker api doesn't print to stdout
        cmd = ["docker", "run", "--network", "host", VALIDATOR_CONTAINER, 
               "rosrun", f"test_{test_name}", f"test_{test_name}"]

        subprocess.run(cmd, check=True, text=True)
    except:
        print("TESTS FAILED")
        assert False
    finally:
        model.stop()
