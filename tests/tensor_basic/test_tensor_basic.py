# Runs inside the docker container

import subprocess
import os
from pathlib import Path
import model

def func(x):
    return x + 1

def test_true():
    assert func(3) == 4

def test_false():
    assert func(3) == 5

def test_app():
    os.environ["MLFLOW_TRACKING_URI"] = "http://127.0.0.1:8080"

    # start server in background
    cmd = [
        "mlflow",
        "server",
        "--host",
        "127.0.0.1",
        "--port",
        "8080"
    ]
    print(' '.join(cmd))
    subprocess.Popen(cmd)

    # log model
    model.run()

    # generate dockerfile
    cmd = [
        "./app.py",
        "generate-dockerfile",
        "--output-directory",
        "artifacts",
        "tensor-basic",
        "1"
    ]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True, text=True)

    expected = Path(Path.cwd() / "integration_pkg" / "tensor_basic_msgs" / "srv" / "tensor_basic.srv").read_text()
    actual = (Path.cwd() / "artifacts" / "tensor_basic_msgs" / "srv" / "tensor_basic.srv").read_text()
    assert expected == actual
