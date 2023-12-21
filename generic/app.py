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


def unpack_type(sig, name): 
    ros_dtype = None
    shape = None
    
    sig = json.loads(sig)[0]
    
    if sig["type"] == "tensor":
        ros_dtype = TENSOR_TO_ROS[sig["tensor-spec"]["dtype"]]
        shape = sig["tensor-spec"]["shape"]
        
    # TODO: fixed size cases
    return Msg(ros_dtype + "[]", name, shape) 

def gen_msg_files():
    cfg = ConfigObj(os.path.join(__location__, CONFIG_NAME))
    remote_uri = cfg['MODEL_URI']
    model_name = remote_uri.split('/')[-2]
    
    pyfunc_model = mlflow.pyfunc.load_model(model_uri=os.path.join(__location__, LOCAL_URI))
    sig = pyfunc_model._model_meta._signature.to_dict()
    
    req = unpack_type(sig=sig["inputs"], name=model_name + "-req")
    res = unpack_type(sig=sig["outputs"], name=model_name + "-res")
    
    service = Srv(req, res)

    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath=os.path.join(__location__, "templates")),
        autoescape=jinja2.select_autoescape,
        undefined=jinja2.StrictUndefined
    )
    
    template = env.get_template("service.srv")
    
    with open(os.path.join(__location__, "rospkg", "srv", f"{model_name}.srv"), "w") as f:
        f.write(template.render(asdict(service)))


def gen_file_structure():
    print("todo")

def gen_interface():
    print("todo")


def main():    
    # gen_file_structure()
    gen_msg_files()
    # gen_interface()


if __name__ == "__main__":
    main()
