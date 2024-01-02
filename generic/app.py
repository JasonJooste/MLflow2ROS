import os
from os.path import join
import mlflow
import jinja2
from configobj import ConfigObj
import json
from dataclasses import dataclass, asdict
from pathlib import Path

LOCAL_URI = "dockerfile/model_dir"
CONFIG_NAME = "config.cfg"
# Dict to convert tensor data types (i.e. numpy) to ROS data types
# TODO: Finish filling out with:
# - https://numpy.org/devdocs/user/basics.types.html
# - http://wiki.ros.org/msg
TENSOR_TO_ROS = {
  "float64": "float64",
  "int64": "int64"
}

# absolute path to the location of this script
__location__ = os.path.realpath(
  join(os.getcwd(), os.path.dirname(__file__)))


@dataclass
class Msg():
  type: str
  name: str
  shape: str
  
@dataclass
class Srv(): 
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


def gen_msg(env, model_name, msgs_dir):
  pyfunc_model = mlflow.pyfunc.load_model(model_uri=join(__location__, LOCAL_URI))
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
    ("exec_launch_template.launch", join(exec_dir, "launch", f"{model_name}.launch"))
  )
  
  render_data = {"model_name": model_name, "msg_pkg": msg_pkg}

  for template_name, output_path in files:
    template = env.get_template(template_name)
    
    with open(output_path, "w") as f:
      f.write(template.render(render_data))


def main():
  # load jinja environment and read config
  env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(searchpath=join(__location__, "templates")),
    autoescape=jinja2.select_autoescape,
    undefined=jinja2.StrictUndefined
  )  
  cfg = ConfigObj(join(__location__, CONFIG_NAME))
  remote_uri = cfg['MODEL_URI']
  model_name = remote_uri.split('/')[-2].replace("-", "_")
  msg_pkg = model_name + "_msgs"

  exec_dir = join(__location__, "rospkg", model_name)
  msgs_dir = join(__location__, "rospkg", msg_pkg)
  
  # create directories
  Path(join(exec_dir, "launch")).mkdir(parents=True, exist_ok=True)
  Path(join(exec_dir, "scripts")).mkdir(parents=True, exist_ok=True)
  Path(join(msgs_dir, "srv")).mkdir(parents=True, exist_ok=True)

  srv = gen_msg(env, model_name, msgs_dir)
  gen_exec(env, model_name, msg_pkg, srv, exec_dir)
  gen_pkg(env, model_name, msg_pkg, msgs_dir, exec_dir)


if __name__ == "__main__":
  main()
