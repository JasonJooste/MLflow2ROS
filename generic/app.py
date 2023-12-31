import os
import mlflow
import jinja2
from configobj import ConfigObj
import rospy
import numpy as np
import json
from dataclasses import dataclass, asdict

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

def gen_msg_files(env, model_name):
    pyfunc_model = mlflow.pyfunc.load_model(model_uri=os.path.join(__location__, LOCAL_URI))
    sig = pyfunc_model._model_meta._signature.to_dict()

    req_msg_name = model_name + "_req"
    res_msg_name = model_name + "_res"
    
    req = unpack_schema(sig=sig["inputs"], name=req_msg_name)
    res = unpack_schema(sig=sig["outputs"], name=res_msg_name)
    
    service = Srv(req, res)
    
    template = env.get_template("service.srv")
    
    with open(os.path.join(__location__, "rospkg", f"{model_name}_msgs", "srv", f"{model_name}.srv"), "w") as f:
        f.write(template.render(asdict(service)))

    return service

def gen_file_structure():
    print("todo")

def gen_exec(env, model_name, srv):
    template = env.get_template("exec")

    msg_pkg_name = model_name + "_msgs"

    render_data = asdict(srv)
    render_data["model_name"] = model_name
    render_data["msg_pkg"] = msg_pkg_name

    with open(os.path.join(__location__, "rospkg", model_name, "scripts", model_name), "w") as f:
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
    
    # gen_file_structure()
    srv = gen_msg_files(env, model_name)
    gen_exec(env, model_name, srv)


if __name__ == "__main__":
    main()
