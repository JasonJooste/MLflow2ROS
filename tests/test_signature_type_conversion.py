""" Test conversion between mlflow signatures and ros message types"""
# pylint: disable=redefined-outer-name,unused-argument

import sys
from pathlib import Path

test_dir = Path(__file__).parent
root_dir = test_dir.parent
sys.path.append(str(root_dir))

# pylint: disable=import-error,wrong-import-position,wrong-import-order
import app


def test_tensor_basic():
    sig_dict = {
        "inputs": [{"type": "tensor", "tensor-spec": {"dtype": "float64", "shape": [-1, 4]}}],
        "outputs": [{"type": "tensor", "tensor-spec": {"dtype": "int64", "shape": [-1]}}],
        "params": None,
    }
    expected_req=app.Msg(
            ros_dtype="float64[]",
            shape=[-1, 4],
            base_dtype="float64",
        )
    expected_res=app.Msg(
            ros_dtype="int64[]",
            shape=[-1],
            base_dtype="int64",
        )
    actual_req = app.unpack_schema(sig_dict["inputs"][0])
    actual_res = app.unpack_schema(sig_dict["outputs"][0])
    assert expected_req == actual_req
    assert expected_res == actual_res
