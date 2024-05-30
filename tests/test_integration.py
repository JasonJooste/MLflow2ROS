import os
import subprocess
import time
import sys
from pathlib import Path

import docker
import pytest
import mlflow

from sklearn_models import *

test_dir = Path(__file__).parent
root_dir = test_dir.parent
sys.path.append(str(root_dir))

import app


ALL_LOGGED_MODELS = ["sklearn_logged_knn", "sklearn_logged_logreg"]

##### Model tests #####

@pytest.mark.parametrize("logged_model", ALL_LOGGED_MODELS)
def test_model_image_generation(request, mlflow_server, logged_model, tmp_path):
    """ Test the the image of a model can be generated and run in-image testing"""
    # Load the model fixture with pytest
    logged_model = request.getfixturevalue(logged_model)
    model_tag = f"{logged_model}_testimage"
    app.make_image(model_name=logged_model, model_ver=1, tag=model_tag, out_dir=tmp_path, env_manager="virtualenv", tracking_uri=mlflow_server, run_tests=True)
