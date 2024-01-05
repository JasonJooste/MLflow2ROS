#! /usr/bin/env python3

import json
import subprocess
import dataclasses
from pathlib import Path
from typing import Optional

import jinja2
import mlflow
import rich
import typer
from typing_extensions import Annotated

# Dict to convert tensor data types (i.e. numpy) to ROS data types
TENSOR_TO_ROS = {"float64": "float64", "int64": "int64"}

app = typer.Typer()


@dataclasses.dataclass
class Msg:
    type: str
    name: str
    shape: str


@dataclasses.dataclass
class Srv:
    request: Msg
    response: Msg


def unpack_schema(sig, name):
    sig = json.loads(sig)[0]

    if sig["type"] == "tensor":
        ros_dtype = TENSOR_TO_ROS[sig["tensor-spec"]["dtype"]]
        shape = sig["tensor-spec"]["shape"]
    elif sig["type"] == "dataframe":
        raise NotImplementedError("Model signature type `dataframe` is not supported")
    elif sig["type"] == "column":
        raise NotImplementedError("Model signature type `column` is not supported")
    
    if "params" in sig:
        raise NotImplementedError("Model signature with parameters is not supported")

    return Msg(ros_dtype + "[]", name, shape)


def filter_model_name(model_name):
    # currently only removes dashes
    return model_name.replace("-", "_")


def get_model_uri(model_name, model_ver):
    client = mlflow.client.MlflowClient()
    return client.get_model_version_download_uri(model_name, model_ver)


def gen_msg(env, model_name, model_ver, msgs_dir):
    # get model signature
    model_uri = get_model_uri(model_name, model_ver)
    sig = mlflow.models.get_model_info(model_uri).signature.to_dict()

    clean_name = filter_model_name(model_name)

    req_msg_name = clean_name + "_req"
    res_msg_name = clean_name + "_res"
    req = unpack_schema(sig["inputs"], req_msg_name)
    res = unpack_schema(sig["outputs"], res_msg_name)
    service = Srv(req, res)

    template = env.get_template("service.srv")
    target_file = msgs_dir/"srv"/f"{clean_name}.srv"
    target_file.write_text(template.render(dataclasses.asdict(service)))

    return service


def gen_exec(env, model_name, msg_pkg, srv, exec_dir):
    clean_name = filter_model_name(model_name)

    render_data = dataclasses.asdict(srv)
    render_data["model_name"] = clean_name
    render_data["msg_pkg"] = msg_pkg

    template = env.get_template("exec.py")
    target_file = exec_dir/"scripts"/f"{clean_name}.py"
    target_file.write_text(template.render(render_data))


def gen_pkg(env, model_name, msg_pkg, msg_dir, exec_dir):
    clean_name = filter_model_name(model_name)

    # pairs of (<template name>, <output path>)
    files = (
        ("msgs_cmake_template.txt", msg_dir/"CMakeLists.txt"),
        ("msgs_package_template.xml", msg_dir/"package.xml"),
        ("exec_cmake_template.txt", exec_dir/"CMakeLists.txt"),
        ("exec_package_template.xml", exec_dir/"package.xml"),
        ("exec_launch_template.launch", exec_dir/"launch"/f"{clean_name}.launch"),
    )

    render_data = {"model_name": clean_name, "msg_pkg": msg_pkg}

    for template_name, output_path in files:
        template = env.get_template(template_name)
        with open(output_path, "w", encoding="UTF-8") as f:
            f.write(template.render(render_data))


@app.command()
def generate_dockerfile(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(help="Version of the model as listed on the MLFlow Model Registry"),
    ],
    rospkg_directory: Annotated[
        Optional[str],
        typer.Option(
            help="Location of the ROS package that will be run on the Docker image"
        ),
    ] = "rospkg",
    output_directory: Annotated[
        Optional[str], typer.Option(help="Folder containing the generated files")
    ] = "dockerfile",
):
    """
    Creates the Dockerfile used to build the image containing the model and its ROS package. 
    The model is downloaded from the MLFLow model registry as part of the process.  
    """
    # get mlflow to generate their docker image
    subprocess.run(
        [
            "mlflow",
            "models",
            "generate-dockerfile",
            "--model-uri",
            get_model_uri(model_name, model_ver),
            "--output-directory",
            output_directory,
        ],
        check=True,
        text=True,
    )

    dfile_path = Path(output_directory)/"Dockerfile"

    with open(Path(output_directory)/"Dockerfile", "r", encoding="UTF-8") as reader:
        dfile_contents = reader.readlines()

    with open(dfile_path, "w", encoding="UTF-8") as writer:
        # edit mlflow dockerfile
        for line in dfile_contents:
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
            loader=jinja2.FileSystemLoader(searchpath=Path.cwd()/"templates"),
            autoescape=jinja2.select_autoescape,
            undefined=jinja2.StrictUndefined,
        )

        render_data = {
            "ubuntu_distro": "focal",
            "ros_distro": "noetic",
            "model_name": filter_model_name(model_name),
            "rospkg_dir": rospkg_directory,
        }

        template = env.get_template("addendum.Dockerfile")
        writer.write(template.render(render_data))


@app.command()
def generate_rospkg(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(help="Version of the model as listed on the MLFlow Model Registry"),
    ],
    output_directory: Annotated[
        Optional[str], typer.Option(help="Folder containing the generated files")
    ] = "rospkg",
):
    """
    Creates the source files for the model's ROS node and message files.
    """
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath=Path.cwd()/"templates"),
        autoescape=jinja2.select_autoescape,
        undefined=jinja2.StrictUndefined,
    )

    clean_name = filter_model_name(model_name)
    msg_pkg = clean_name + "_msgs"
    exec_dir = Path.cwd()/output_directory/clean_name
    msgs_dir = Path.cwd()/output_directory/msg_pkg

    # create directories
    Path(exec_dir/"launch").mkdir(parents=True, exist_ok=True)
    Path(exec_dir/"scripts").mkdir(parents=True, exist_ok=True)
    Path(msgs_dir/"srv").mkdir(parents=True, exist_ok=True)

    srv = gen_msg(env, model_name, model_ver, msgs_dir)
    gen_exec(env, model_name, msg_pkg, srv, exec_dir)
    gen_pkg(env, model_name, msg_pkg, msgs_dir, exec_dir)

    print(f"Generated ROS package in directory {output_directory}")


@app.command()
def make_image(
    model_name: Annotated[
        str, typer.Argument(help="Name of model as listed on the MLFlow Model Registry")
    ],
    model_ver: Annotated[
        int,
        typer.Argument(help="Version of the model as listed on the MLFlow Model Registry"),
    ],
    tag: Annotated[
        Optional[str], typer.Option(help="Name of the generated image")
    ] = None,
):
    """
    Builds a Docker image containing the generated ROS node.
    """
    generate_dockerfile(model_name, model_ver)
    generate_rospkg(model_name, model_ver)

    # create docker build command
    cmd = [
        "docker",
        "build",
        "--file",
        str(Path.cwd()/"dockerfile"/"Dockerfile"),
        "--target",
        "prod",
    ]
    if not tag is None:
        cmd = cmd + ["--tag", tag]
    cmd.append(str(Path.cwd()))

    rich.print(f"Running command: '{' '.join(cmd)}'")

    subprocess.run(cmd, check=True, text=True)


if __name__ == "__main__":
    app()
