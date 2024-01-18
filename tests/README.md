# Integration tests 
## How they work
Different tests are stored in `integration/tests`. There is only one test right now, which is `tensor_basic`. Each test consists of a model, the source code for the test node, and a Dockerfile for the test node. 

The test runner is `integration_test.py` in the root directory. The integration test process can be summed up as the following:
1. Save any environment variables that need to be restored after tests end.
2. Start a container with the MLflow server and another container to run roscore. 
3. Go through every directory in `integration/tests` where:
    1. The test node container is build. 
    2. The `model.py` is run and the model is logged to the mlflow server container.
    3. MLflow2ROS is run with the model that was just logged and it creates the docker image for the model. 
    4. The generated docker image and test node image are then run together. The test image then sends a request to the model image and examines the reponse. 
    5. Repeate for the other tests (note: there is currently only one test).

## Issues
The model is run locally, so it is currently not possible to test models with different dependencies. 