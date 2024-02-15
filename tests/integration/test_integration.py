import docker
import pytest

import time

import subprocess
from pathlib import Path

import app
import os
from models import model_sklearn_tensor_basic, model_tf_floats

VALIDATOR_CONTAINER = "integration_test_validator"
TESTING_TRACKING_URI = "http://127.0.0.1:55555"

@pytest.fixture
def override_tracking_uri(scope="session"):
    # save current tracking uri to be restored once tests conclude
    old_tracking_uri = None
    if "MLFLOW_TRACKING_URI" in os.environ:
        old_tracking_uri = os.environ["MLFLOW_TRACKING_URI"]
    os.environ["MLFLOW_TRACKING_URI"] = TESTING_TRACKING_URI

    yield old_tracking_uri

    if old_tracking_uri is None:
        # remove env var
        del os.environ["MLFLOW_TRACKING_URI"]
    else:
        # restore tracking uri
        os.environ["MLFLOW_TRACKING_URI"] = old_tracking_uri


@pytest.fixture
def docker_client(scope="session"):
    return docker.from_env()


@pytest.fixture
def build_validator_container(scope="session"):
    print("Building validator container")

    subprocess.run(["docker", "build", "--tag", VALIDATOR_CONTAINER, Path.cwd()/"tests"/"integration"/"test_nodes"], check=True, text=True)


@pytest.fixture
def roscore_container(docker_client, build_validator_container, scope="session"):
    container = docker_client.containers.run(
        VALIDATOR_CONTAINER, "roscore", network="host", detach=True, auto_remove=True)
    yield container
    container.stop()
    

@pytest.fixture
def mlflow_server_container(docker_client, scope="session"):
    container = docker_client.containers.run(
        "ghcr.io/mlflow/mlflow:v2.9.2", "mlflow server --host 127.0.0.1 --port 55555",
        network="host", detach=True, auto_remove=True)
    yield container
    container.stop()


def make_image_and_run_tests(docker_client, test_name, model_name):
    # run app to build image
    app.make_image(model_name=model_name, model_ver=1, tag=model_name, output_directory=Path("tests_generated_files") / test_name)

    model = docker_client.containers.run(model_name, network="host", detach=True, auto_remove=True)

    # wait a bit and make sure the model container hasn't crashed. It would be better to make another test that checks if the 
    # model container launches the model correctly. 
    time.sleep(5)
    model.reload()
    assert(model.status != "exited")

    try:
        # Start validator container. Must use subprocess since the docker api doesn't print to stdout
        # The name of the ros node to run depends on what is coded up in the test container src files.
        cmd = ["docker", "run", "--network", "host", VALIDATOR_CONTAINER, 
               "rosrun", f"{test_name}_node", f"{test_name}_node.py"] 
        
        # The validator container will exit with a non zero status if the tests in it fail
        subprocess.run(cmd, check=True, text=True)
    except subprocess.CalledProcessError:
        print("TESTS FAILED")
        assert False
    finally:
        model.stop()


def test_sklearn_tensor_basic(docker_client, mlflow_server_container, roscore_container, build_validator_container, override_tracking_uri):
    test_name = "sklearn_tensor_basic"
    model_name = f"{test_name}_model"
    model_sklearn_tensor_basic.run_and_log(model_name)
    make_image_and_run_tests(docker_client, test_name, model_name)


def test_tensorflow_floats(docker_client, mlflow_server_container, roscore_container, build_validator_container, override_tracking_uri):
    test_name = "tf_floats"
    model_name = f"{test_name}_model"
    model_tf_floats.run_and_log(model_name)
    make_image_and_run_tests(docker_client, test_name, model_name)
