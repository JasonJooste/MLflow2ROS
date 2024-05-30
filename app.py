#! /usr/bin/env python3

"""
This module operates as a standalone executable which offers a command line 
interface for converting MLFlow models into ROS packages and containerises with Docker.
"""

import dataclasses
import json
import os
import re
import subprocess
from pathlib import Path
from typing import Optional
import shutil

import jinja2
import mlflow
import rich
import typer
import docker
from typing_extensions import Annotated

# Dict to convert tensor data types (i.e. numpy) to ROS data types
TENSOR_TO_ROS = {
    "bool": "bool",
    "int8": "int8",
    "uint8": "uint8",
    "int16": "int16",
    "uint16": "uint16",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "intp": "int64",
    "uintp": "uint64",
    "float16": "float32",
    "float32": "float32",
    "float64": "float64",
}

root_dir = Path(__file__).parent

app = typer.Typer()


# class ModelMetapackage:
        # """ Class to store all info required to make a model metapackage in ros """ 
        # srv: Srv
        # model_name: str  

class LoggedModelError(Exception):
    pass

@dataclasses.dataclass
class Msg:
    """Dataclass to store info that will be inserted into a ROS message"""
    ros_dtype: str
    shape: list
    base_dtype: str

    def asdict(self):
        return dataclasses.asdict(self)


@dataclasses.dataclass
class Srv:
    """Dataclass to store info that will be inserted into a ROS service"""

    request: Msg
    response: Msg

    def asdict(self):
        return dataclasses.asdict(self)


def unpack_schema(sig):
    """Reads a model signature dictionary and returns its corresponding ROS msg info"""
    if sig["type"] == "tensor":
        ros_dtype = TENSOR_TO_ROS[sig["tensor-spec"]["dtype"]]
        shape = sig["tensor-spec"]["shape"]
    return Msg(ros_dtype + "[]", shape, sig["tensor-spec"]["dtype"])


def filter_model_name(model_name):
    """Removes invalid symbols from model names. Currently only removes dashes"""
    return model_name.replace("-", "_")


def get_model_uri(model_name, model_ver):
    """Retrieves the model uri from the tracking server"""
    client = mlflow.client.MlflowClient()
    return client.get_model_version_download_uri(model_name, model_ver)


def check_signature(sig):
    """ Check that the signature is able to be processed by MLFlow2ROS"""
    # Check that the signature type is supported
    # Check that both input and output signatures are present
    if "inputs" not in sig and "outputs" not in sig:
        raise LoggedModelError("Model has been logged without a signature. MLFLow2ROS needs an input/output signature for the model to construct the input/output ROS messages. See https://mlflow.org/docs/latest/model/signatures.html#id1")
    elif "inputs" not in sig:
        raise LoggedModelError("Model has been logged without an input signature. Perhaps you used 'infer_signature' incorrectly? Make sure to include both input and output data when inferring the signature")
    elif "outputs" not in sig:
        raise LoggedModelError("Model has been logged without an output signature. Perhaps you used 'infer_signature' incorrectly? Make sure to include both input and output data when inferring the signature")
    for s in ["inputs", "outputs"]:
        if len(sig[s]) != 1:
            raise NotImplementedError(f"Only {s[:-1]} signatures of length 1 are supported. This signature has length {len(sig[s])}")
        if sig[s][0]["type"] == "dataframe":
            raise NotImplementedError("Model signature type `dataframe` is not supported")
        elif sig[s][0]["type"] == "column":
            raise NotImplementedError("Model signature type `column` is not supported")
    if "params" in sig:
        raise NotImplementedError("Model signature with parameters is not supported")

def gen_msg(model_name, model_ver):
    """Writes a ROS .srv file for a model"""
    # get model signature
    model_uri = get_model_uri(model_name, model_ver)
    # NOTE: This could be moved out of this fn
    sig = mlflow.models.get_model_info(model_uri).signature.to_dict()
    # The values of the signature dict are stored as strings and need to be processed
    sig_dict = {k:json.loads(v) for k,v in sig.items() if v is not None}
    check_signature(sig_dict)
    clean_name = filter_model_name(model_name)
    # We currently only support input/output lengths of 1
    request = unpack_schema(sig_dict["inputs"][0])
    response = unpack_schema(sig_dict["outputs"][0])
    return Srv(request,response)


def write_msg(env, service, clean_name, out_dir):
    template = env.get_template("model_msgs/srv/__model_name__.srv.tmpl")
    target_file = out_dir / "model_msgs" / "srv" / f"{clean_name}.srv"
    # target_file.write_text(template.render(dataclasses.asdict(service)))
    target_file.parent.mkdir(exist_ok=True, parents=True)
    target_file.write_text(template.render(service.asdict()))


def write_server(env, clean_name, srv, out_dir):
    """Writes a ROS node server package for the model"""
    render_data = srv.asdict() | {"model_name": clean_name, "msg_pkg":  "model_msgs"}
    template = env.get_template("model_server/scripts/serve_model.py.tmpl")
    target_file = out_dir / "model_server" / "scripts" / f"serve_model.py"
    target_file.parent.mkdir(exist_ok=True, parents=True)
    target_file.write_text(template.render(render_data))


def write_pkg(env, srv, clean_name, out_dir):
    """Creates the ROS package around the node server and message packages"""
    files = ["model_msgs/CMakeLists.txt.tmpl",
             "model_msgs/package.xml.tmpl",
             "model_server/CMakeLists.txt.tmpl",
             "model_server/package.xml.tmpl",
             "model_server/launch/model_server.launch.tmpl",
             "model_server/launch/test_model.test.tmpl",
             "model_server/scripts/test_model.py.tmpl"]
    render_data = srv.asdict() | {"model_name": clean_name, "msg_pkg":  "model_msgs"}

    for template_path in files:
        template = env.get_template(template_path)
        assert template_path[-5:] == ".tmpl"
        out_path = out_dir / template_path[:-5]
        out_path.parent.mkdir(exist_ok=True, parents=True)
        out_path.write_text(template.render(render_data))


def write_dockerfile(clean_name, mlflow_dockerfile_path):
    """Changes the MLFLow generated dockerfile such that ROS and the
    ROS package files are installed"""
    contents = mlflow_dockerfile_path.read_text()
    # rename mlflow generated Dockerfile
    mlflow_dockerfile_path.rename(mlflow_dockerfile_path.parent / "mlflow_base.dockerfile")
    # Add `as base` to the initial build stage
    assert not re.search("FROM ubuntu:20.04", contents) is None
    contents = re.sub("FROM ubuntu:20.04", "FROM ros:noetic as base", contents)
    # Remove the entrypoint command in the dockerfile as we will define a new one later
    assert not re.search(r"ENTRYPOINT(.*)(?=\n)", contents) is None
    contents = re.sub(r"ENTRYPOINT(.*)(?=\n)", "", contents)
    # Edit the COPY command to work with our build context
    assert not re.search(r"(?<=COPY )(.*)(?=model_dir)", contents) is None
    contents = re.sub(
        r"(?<=COPY )(.*)(?=model_dir)", f"docker/", contents
    )
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath=root_dir / "templates"),
        autoescape=jinja2.select_autoescape,
        undefined=jinja2.StrictUndefined,
    )

    render_data = {
        "ros_distro": "noetic",
        "model_name": clean_name,
    }

    template = env.get_template("Dockerfile_addendum.tmpl")
    contents = contents + template.render(render_data)

    mlflow_dockerfile_path.write_text(contents)


def test_image(tag, clean_name):
    """ Trigger testing in an image"""
    test_cmd = [
            "docker",
            "run",
            "--network",
            "host",
            "--entrypoint",
            "",
            tag,
            "bash",
            "-c",
            f". /activate_ros_env.bash; . /activate_mlflow_env.bash; rostest {clean_name} test_model.test"]
    subprocess.run(test_cmd, check=True, text=True)

@app.command()
def generate_rospkg(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(
            help="Version of the model as listed on the MLFlow Model Registry"
        ),
    ],
    out_dir: Annotated[
        Optional[str], typer.Option(help="Folder for the generated files")
    ] = "rospkg",
    tracking_uri: Annotated[Optional[str], typer.Option(help="Uri of the MLFLow tracking server")] = "http://127.0.0.1:5000"
):
    """
    Creates the source files for the model's ROS node and message files.
    """
    out_dir = Path(out_dir)
    mlflow.set_tracking_uri(tracking_uri)
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath= root_dir / "templates"),
        autoescape=jinja2.select_autoescape,
        undefined=jinja2.StrictUndefined,
    )

    clean_name = filter_model_name(model_name)
    srv = gen_msg(model_name, model_ver)
    write_msg(env, srv, clean_name, out_dir)
    write_server(env, clean_name, srv, out_dir)
    write_pkg(env, srv, clean_name, out_dir)

    rich.print(f"Generated ROS package in directory {out_dir}")


@app.command()
def generate_dockerfile(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(
            help="Version of the model as listed on the MLFlow Model Registry"
        ),
    ],
    rospkg_directory: Annotated[
        Optional[str],
        typer.Option(
            help="Location of the ROS package that will be run on the Docker image"
        ),
    ] = None,
    out_dir: Annotated[
        Optional[str], typer.Option(help="Folder containing the generated files")
    ] = "generated_files",
    env_manager: Annotated[
        Optional[str],
        typer.Option(
            help="The environment manager that will be used to install model "
            + "dependencies and run the model. Can be 'local', 'virtualenv' or 'conda'"
        ),
    ] = "virtualenv",
        tracking_uri: Annotated[Optional[str], typer.Option(help="Uri of the MLFLow tracking server")] = "http://127.0.0.1:5000"

):
    """
    Creates the Dockerfile used to build the image containing the model and its ROS package.
    The model is downloaded from the MLFLow model registry as part of the process.
    """
    mlflow.set_tracking_uri(tracking_uri)
    dockerfile_dir = Path(out_dir) / "docker"

    build_rospkg_dir = Path(out_dir) / "rospkg"
    if rospkg_directory is None:
        rospkg_directory = build_rospkg_dir
        rich.print("Custom rospkg not provided. Creating one from the model")
        generate_rospkg(model_name, model_ver, out_dir=rospkg_directory, tracking_uri=tracking_uri)
    else:
        if build_rospkg_dir.exists():
            logging.warning(f"rospkg output directory {build_rospkg_dir} already exists")
        shutil.copy(rospkg_dir, build_rospkg_dir)
    # Generate the base dockerfile with MLFLow
    # NOTE: There should be a way to call this through the python api
    mlflow_model_uri = get_model_uri(model_name, model_ver)
    subprocess.run(
        [
            "mlflow",
            "models",
            "generate-dockerfile",
            "--model-uri",
            mlflow_model_uri,
            "--output-directory",
            dockerfile_dir,
            "--env-manager",
            env_manager,
        ],
        check=True,
        text=True,
    )
    # edit mlflow's dockerfile
    clean_name = filter_model_name(model_name)
    mlflow_dockerfile_path = dockerfile_dir / "Dockerfile"
    write_dockerfile(clean_name, mlflow_dockerfile_path)


@app.command()
def make_image(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(
            help="Version of the model as listed on the MLFlow Model Registry"
        ),
    ],
    tag: Annotated[
        Optional[str], typer.Option(help="Name of the generated image")
    ] = None,
    out_dir: Annotated[
        Optional[str], typer.Option(help="Folder containing the generated files")
    ] = "generated_files",
    env_manager: Annotated[
        Optional[str],
        typer.Option(
            help="The environment manager that will be used to install model "
            + "dependencies and run the model. Can be 'local', 'virtualenv' or 'conda'"
        ),
    ] = "virtualenv",
    tracking_uri: Annotated[Optional[str], typer.Option(help="Uri of the MLFLow tracking server")] = "http://127.0.0.1:5000",
    run_tests: Annotated[Optional[bool], typer.Option(help="Test the generated server node")] = True
):
    """
    Builds a Docker image containing the ROS node for the model.
    The model is downloaded locally in the process.
    """
    out_dir = Path(out_dir).absolute()
    generate_dockerfile(
        model_name,
        model_ver,
        out_dir=out_dir,
        env_manager=env_manager,
        tracking_uri=tracking_uri,
    )
    # Run docker build
    # NOTE: Could do this with python_on_whales to avoid the subprocess call
    cmd = [
        "docker",
        "build",
        "--file",
        str(out_dir / "docker" / "Dockerfile"),
        "--target",
        "prod",
    ]
    if not tag is None:
        cmd = cmd + ["--tag", tag]
    cmd.append(str(out_dir))

    rich.print(f"Running command: '{' '.join(cmd)}'")
    
    subprocess.run(cmd, check=True, text=True)
    

    # Test the generated model
    if run_tests:
        rich.print("Running tests")
        clean_name = filter_model_name(model_name)
        if tag is None:
            raise NotImplementedError("Testing without a tag to reference the image is not yet supported")
        test_image(tag, clean_name)


if __name__ == "__main__":
    app()
