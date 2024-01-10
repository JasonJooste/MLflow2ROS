# Runs on the host

import subprocess
from pathlib import Path

def main():
    # create artifacts folder 
    Path(Path.cwd() / "artifacts").mkdir(exist_ok=True)

    # build test image 
    cmd = [
        "docker",
        "build",
        "--file",
        str(Path.cwd() / "Dockerfile"),
        "--tag",
        "test_tensor_basic",
        str(Path.cwd().parent.parent)
    ]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True, text=True)

    # run the model, log it, then run the app. Exit from docker
    cmd = [
        "docker",
        "run",
        "--mount",
        f"type=bind,src={str(Path.cwd()/'artifacts')},dst=/workspace/artifacts",
        "--entrypoint", # run pytest as entrypoint
        "pytest",
        "test_tensor_basic", # image to run
        "test_tensor_basic.py", # test executable to run
        "--doctest-modules", 
        "--junitxml=artifacts/test_results/test_results.xml", # export results as junit xml
        "--cov=.", # run coverage on files in current directory
        "--cov-report=json:artifacts/test_results/coverage.json", # export coverage report as json and html
        "--cov-report=html:artifacts/test_results/htmlcov"
    ]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True, text=True)

    # build node image with generated files

    # run node container in background

    # run test image with integration tests




if __name__ == "__main__":
    main()