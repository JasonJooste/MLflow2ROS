import mlflow
import pytest
import sklearn.datasets
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier


mlflow.set_tracking_uri("file:///home/jason/Documents/projects/MLflow2ROS/tmp")
iris = sklearn.datasets.load_iris()
x = iris["data"][:, :2]
y = iris["target"]
knn_model = KNeighborsClassifier()
knn_model.fit(x, y)
signature = mlflow.models.infer_signature(x, y)
with mlflow.start_run():
    mlflow.sklearn.log_model(
        knn_model,
        "model",
        signature=signature,
        input_example=x,
        registered_model_name="tmp_knn",
    )
