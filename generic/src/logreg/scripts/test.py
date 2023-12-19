import mlflow

def main():
    pyfunc_model = mlflow.pyfunc.load_model(model_uri=f"/model")
    sig = pyfunc_model._model_meta._signature.to_dict()
    print(sig)
    print(pyfunc_model._model_meta)


if __name__ == "__main__":
  main()