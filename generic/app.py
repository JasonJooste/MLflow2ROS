import os
import mlflow
import jinja2
from configobj import ConfigObj
import rospy
import numpy as np
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
	os.path.join(os.getcwd(), os.path.dirname(__file__)))


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

def gen_msg(env, model_name, msg_pkg, msgs_dir):
	pyfunc_model = mlflow.pyfunc.load_model(model_uri=os.path.join(__location__, LOCAL_URI))
	sig = pyfunc_model._model_meta._signature.to_dict()

	req_msg_name = model_name + "_req"
	res_msg_name = model_name + "_res"
	
	req = unpack_schema(sig=sig["inputs"], name=req_msg_name)
	res = unpack_schema(sig=sig["outputs"], name=res_msg_name)
	
	service = Srv(req, res)
	
	template = env.get_template("service.srv")
	
	with open(os.path.join(msgs_dir, "srv", f"{model_name}.srv"), "w+") as f:
		f.write(template.render(asdict(service)))

	return service

def gen_exec(env, model_name, msg_pkg, srv, exec_dir):
	template = env.get_template("exec")

	render_data = asdict(srv)
	render_data["model_name"] = model_name
	render_data["msg_pkg"] = msg_pkg

	with open(os.path.join(exec_dir, "scripts", model_name), "w") as f:
		f.write(template.render(render_data))

def gen_pkg(env, model_name, msg_pkg, msg_dir, exec_dir):
  render_data = {"model_name": model_name, "msg_pkg": msg_pkg}

	# msg cmake
  template = env.get_template("msgs_cmake_template.txt")
  with open(os.path.join(msg_dir, "CMakeLists.txt"), "w") as f:
    f.write(template.render(render_data))

	# msg package
  template = env.get_template("msgs_package_template.xml")
  with open(os.path.join(msg_dir, "package.xml"), "w") as f:
    f.write(template.render(render_data))

	# exec cmake
  template = env.get_template("exec_cmake_template.txt")
  with open(os.path.join(exec_dir, "CMakeLists.txt"), "w") as f:
    f.write(template.render(render_data))

	# exec package 
  template = env.get_template("exec_package_template.xml")
  with open(os.path.join(exec_dir, "package.xml"), "w") as f:
    f.write(template.render(render_data))

	# exec launch
  template = env.get_template("exec_launch_template.launch")
  with open(os.path.join(exec_dir, "launch", f"{model_name}.launch"), "w") as f:
    f.write(template.render(render_data))

def main():
	env = jinja2.Environment(
		loader=jinja2.FileSystemLoader(searchpath=os.path.join(__location__, "templates")),
		autoescape=jinja2.select_autoescape,
		undefined=jinja2.StrictUndefined
	)
	
	cfg = ConfigObj(os.path.join(__location__, CONFIG_NAME))
	remote_uri = cfg['MODEL_URI']
	model_name = remote_uri.split('/')[-2].replace("-", "_")
	msg_pkg = model_name + "_msgs"

	exec_dir = os.path.join(__location__, "rospkg", model_name)
	msgs_dir = os.path.join(__location__, "rospkg", msg_pkg)
	
	# create directories
	Path(os.path.join(exec_dir, "launch")).mkdir(parents=True, exist_ok=True)
	Path(os.path.join(exec_dir, "scripts")).mkdir(parents=True, exist_ok=True)
	Path(os.path.join(msgs_dir, "srv")).mkdir(parents=True, exist_ok=True)

	srv = gen_msg(env, model_name, msg_pkg, msgs_dir)
	gen_exec(env, model_name, msg_pkg, srv, exec_dir)
	gen_pkg(env, model_name, msg_pkg, msgs_dir, exec_dir)


if __name__ == "__main__":
	main()
