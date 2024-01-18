import docker

import subprocess
from pathlib import Path

import app
import os


class IntegrationTest():
    def __init__(self):
        # change tracking uri
        self.old_tracking_uri = None
        if "MLFLOW_TRACKING_URI" in os.environ:
            self.old_tracking_uri = os.environ["MLFLOW_TRACKING_URI"]
        os.environ["MLFLOW_TRACKING_URI"] = "http://127.0.0.1:55555"

        self.client = docker.from_env()

        print("Spinning up test containers")
        # launch mlflow server image and roscore image
        mlflow_server = self.client.containers.run("ghcr.io/mlflow/mlflow:v2.9.2", "mlflow server --host 127.0.0.1 --port 55555", 
                            network="host", detach=True, auto_remove=True)
        roscore = self.client.containers.run("ros:noetic", "roscore", 
                            network="host", detach=True, auto_remove=True)
        
        self.used_containers = [mlflow_server, roscore]


    def test_all(self):
        tests = Path.cwd() / "tests" / "integration" / "tests"
        # run all tests stored in integration/tests
        for test_path in tests.iterdir():
            test_name = test_path.name
            model_img_name = f"{test_name}_model"
            test_img_name = f"{test_name}_test"

            print("Building test node image")
            self.client.images.build(path=str(test_path / "test_node"), tag=test_img_name, quiet=False)

            # log the model
            subprocess.run(
                ["python", test_path/"model.py"],
                check=True,
                text=True,
            )

            # run app to build image. Build process could probably be optimised....
            app.make_image(model_name=test_path.name, model_ver=1, tag=model_img_name)

            print("Starting model container")
            model = self.client.containers.run(model_img_name, network="host", detach=True, auto_remove=True)

            print("Starting test container")
            try:
                # must use subprocess since the docker api doesn't print to stdout
                subprocess.run(["docker", "run", "--network", "host", test_img_name], check=True, text=True)
            except:
                print("TESTS FAILED")
                self.used_containers.append(model)
                self.teardown()
                exit(1)

            model.stop()
        self.teardown()
        

    def teardown(self):
        print("Removing test containers")
        # close used containers
        for container in self.used_containers:
            container.stop()
        
        if self.old_tracking_uri is None:
            # remove env var
            del os.environ["MLFLOW_TRACKING_URI"]
        else:
            # restore tracking uri
            os.environ["MLFLOW_TRACKING_URI"] = self.old_tracking_uri


if __name__ == "__main__":
    IntegrationTest().test_all()