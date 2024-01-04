#!/usr/bin/python3

import json
import os
from dataclasses import asdict, dataclass
from os.path import join
from pathlib import Path
from typing_extensions import Annotated
from typing import Optional
from rich import print

import subprocess
import typer
import jinja2
import mlflow

# Dict to convert tensor data types (i.e. numpy) to ROS data types
# TODO: Finish filling out with:
# - https://numpy.org/devdocs/user/basics.types.html
# - http://wiki.ros.org/msg
TENSOR_TO_ROS = {"float64": "float64", "int64": "int64"}

# absolute path to the location of this script
__location__ = os.path.realpath(join(os.getcwd(), os.path.dirname(__file__)))

app = typer.Typer()

@dataclass
class Msg:
    type: str
    name: str
    shape: str


@dataclass
class Srv:
    request: Msg
    response: Msg


def unpack_schema(sig, name):
    ros_dtype = None
    shape = None

    sig = json.loads(sig)[0]

    if sig["type"] == "tensor":
        ros_dtype = TENSOR_TO_ROS[sig["tensor-spec"]["dtype"]]
        shape = sig["tensor-spec"]["shape"]

    # TODO: fixed size cases
    return Msg(ros_dtype + "[]", name, shape)


def gen_msg(env, model_uri, model_name, msgs_dir):
    pyfunc_model = mlflow.pyfunc.load_model(model_uri)
    sig = pyfunc_model._model_meta._signature.to_dict()

    req_msg_name = model_name + "_req"
    res_msg_name = model_name + "_res"

    req = unpack_schema(sig["inputs"], req_msg_name)
    res = unpack_schema(sig["outputs"], res_msg_name)

    service = Srv(req, res)

    template = env.get_template("service.srv")

    with open(join(msgs_dir, "srv", f"{model_name}.srv"), "w+") as f:
        f.write(template.render(asdict(service)))

    return service


def gen_exec(env, model_name, msg_pkg, srv, exec_dir):
    render_data = asdict(srv)
    render_data["model_name"] = model_name
    render_data["msg_pkg"] = msg_pkg

    template = env.get_template("exec")

    with open(join(exec_dir, "scripts", model_name), "w") as f:
        f.write(template.render(render_data))


def gen_pkg(env, model_name, msg_pkg, msg_dir, exec_dir):
    # pairs of (<template name>, <output path>)
    files = (
        ("msgs_cmake_template.txt", join(msg_dir, "CMakeLists.txt")),
        ("msgs_package_template.xml", join(msg_dir, "package.xml")),
        ("exec_cmake_template.txt", join(exec_dir, "CMakeLists.txt")),
        ("exec_package_template.xml", join(exec_dir, "package.xml")),
        (
            "exec_launch_template.launch",
            join(exec_dir, "launch", f"{model_name}.launch"),
        ),
    )

    render_data = {"model_name": model_name, "msg_pkg": msg_pkg}

    for template_name, output_path in files:
        template = env.get_template(template_name)

        with open(output_path, "w") as f:
            f.write(template.render(render_data))


def parse_model_name(remote_uri):
    # might not work with local uri
    return remote_uri.split("/")[-2].replace("-", "_")


@app.command()
def generate_dockerfile(model_uri: Annotated[str, typer.Argument(help="Location of the MLFLow model")],
                        rospkg_directory: Annotated[Optional[str], typer.Argument(
                            help="Location of the ROS package that will be run on the Docker image")] = "rospkg", 
                        output_directory: Annotated[Optional[str], typer.Argument(
                            help="Folder containing the generated files")] = "dockerfile"):
    """
    Creates the Dockerfile used to build the image containing the model and its ROS package.
    """
    # get mlflow to generate their docker image
    subprocess.run(["mlflow", "models", "generate-dockerfile", "--model-uri", model_uri, 
                    "--output-directory", output_directory], check=True, text=True)
    
    with open(join(output_directory, "Dockerfile"), "r") as reader:
        dockerfile = reader.readlines()

    with open(join(output_directory, "Dockerfile"), "w") as writer:
        # edit mlflow dockerfile
        for line in dockerfile:
            if "FROM ubuntu" in line:
                # Add `as base` to the initial build stage
                line = line.rstrip() + " as base\n"
            elif "ENTRYPOINT" in line:
                # Remove the entrypoint command in the dockerfile as we will define a new one later
                continue
            elif "model_dir" in line: 
                # Edit the COPY command to work with our build context
                line = f"COPY {output_directory}/model_dir /opt/ml/model\n"

            writer.write(line)
    
        # add dockerfile addendum
        env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(searchpath=join(__location__, "templates")),
            autoescape=jinja2.select_autoescape,
            undefined=jinja2.StrictUndefined,
        )   

        render_data = {"ubuntu_distro": "focal", "ros_distro": "noetic", "model_name": parse_model_name(model_uri),
                       "rospkg_dir": rospkg_directory}
        
        template = env.get_template("addendum.Dockerfile")
        writer.write(template.render(render_data))


@app.command()
def generate_rospkg(model_uri: Annotated[str, typer.Argument(help="Location of the MLFLow model")], 
                    output_directory: Annotated[Optional[str], typer.Argument(help="Folder containing the generated files")] = "rospkg"):
    """
    Creates the source files for the model's ROS node and message files.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath=join(__location__, "templates")),
        autoescape=jinja2.select_autoescape,
        undefined=jinja2.StrictUndefined,
    )

    model_name = parse_model_name(model_uri)
    msg_pkg = model_name + "_msgs"

    exec_dir = join(__location__, output_directory, model_name)
    msgs_dir = join(__location__, output_directory, msg_pkg)

    # create directories
    Path(join(exec_dir, "launch")).mkdir(parents=True, exist_ok=True)
    Path(join(exec_dir, "scripts")).mkdir(parents=True, exist_ok=True)
    Path(join(msgs_dir, "srv")).mkdir(parents=True, exist_ok=True)

    srv = gen_msg(env, model_uri, model_name, msgs_dir)
    gen_exec(env, model_name, msg_pkg, srv, exec_dir)
    gen_pkg(env, model_name, msg_pkg, msgs_dir, exec_dir)


@app.command()
def make_image(model_uri: Annotated[str, typer.Argument(help="Location of the MLFLow model")],
               image_name: Annotated[Optional[str], typer.Argument()] = None):
    """
    Builds a Docker image containing the generated ROS node.
    """
    generate_dockerfile(model_uri)
    generate_rospkg(model_uri)

    # create docker build command
    cmd = ["docker", "build", "--file", join(__location__, "dockerfile", "Dockerfile"), "--target", "prod"]

    if not (image_name is None):
        cmd = cmd + ["--tag", image_name]

    cmd.append(__location__)

    print(f"Running command: '{' '.join(cmd)}'")

    subprocess.run(cmd, check=True, text=True)
    

if __name__ == "__main__":
    app()
