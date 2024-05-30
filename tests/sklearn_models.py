import mlflow
import pytest
import sklearn.datasets
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier

##### Model fixtures #####


# These are borrowed from MLFlow
@pytest.fixture(scope="session")
def iris_data():
    iris = sklearn.datasets.load_iris()
    x = iris.data[:, :2]
    y = iris.target
    # NOTE: This is mutable and could (unlikely) cause issues with the session context
    return x, y


@pytest.fixture(scope="session")
def sklearn_knn_model(iris_data):
    x, y = iris_data
    knn_model = KNeighborsClassifier()
    knn_model.fit(x, y)
    return knn_model


@pytest.fixture(scope="session")
def sklearn_logreg_model(iris_data):
    x, y = iris_data
    linear_lr = LogisticRegression()
    linear_lr.fit(x, y)
    return linear_lr


@pytest.fixture(scope="session")
def sklearn_logged_knn(mlflow_server, iris_data, sklearn_knn_model):
    name = "sklearn_knn_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(iris_x, iris_y)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_knn_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


@pytest.fixture(scope="session")
def sklearn_logged_logreg(mlflow_server, iris_data, sklearn_logreg_model):
    name = "sklearn_logreg_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(iris_x, iris_y)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_logreg_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


@pytest.fixture(scope="session")
def sklearn_logged_knn_no_input_sig(mlflow_server, iris_data, sklearn_knn_model):
    name = "sklearn_knn_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(None, iris_y)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_knn_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


@pytest.fixture(scope="session")
def sklearn_logged_logreg_no_input_sig(mlflow_server, iris_data, sklearn_logreg_model):
    name = "sklearn_logreg_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(None, iris_y)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_logreg_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


@pytest.fixture(scope="session")
def sklearn_logged_knn_no_output_sig(mlflow_server, iris_data, sklearn_knn_model):
    name = "sklearn_knn_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(iris_x)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_knn_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


@pytest.fixture(scope="session")
def sklearn_logged_logreg_no_output_sig(mlflow_server, iris_data, sklearn_logreg_model):
    name = "sklearn_logreg_model"
    iris_x, iris_y = iris_data
    signature = mlflow.models.infer_signature(iris_x)
    with mlflow.start_run():
        mlflow.sklearn.log_model(
            sklearn_logreg_model,
            "model",
            signature=signature,
            input_example=iris_x,
            registered_model_name=name,
        )
    return name


# Model with column signature

# Model with tensor signature

# Model with the other signature

# Log with input example and signature

# Log with incorrect input signature

# Log with without input signature

# Log with incorrect environment

# Log without environment
