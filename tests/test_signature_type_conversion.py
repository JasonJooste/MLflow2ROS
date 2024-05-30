import sys
from pathlib import Path

test_dir = Path(__file__).parent
root_dir = test_dir.parent
sys.path.append(str(root_dir))

import app


def test_tensor_basic():
    sig_dict = {
        "inputs": '[{"type": "tensor", "tensor-spec": {"dtype": "float64", "shape": [-1, 4]}}]',
        "outputs": '[{"type": "tensor", "tensor-spec": {"dtype": "int64", "shape": [-1]}}]',
        "params": None,
    }
    model_name = "tensor_basic"
    expected = app.Srv(
        request=app.Msg(
            ros_dtype="float64[]",
            shape=[-1, 4],
            base_dtype="float64",
        ),
        response=app.Msg(
            ros_dtype="int64[]",
            shape=[-1],
            base_dtype="int64",
        ),
    )
    assert app.sig_to_srv(model_name, sig_dict) == expected
