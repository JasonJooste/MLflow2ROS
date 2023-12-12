### Running the container
`docker/run.bash`

### Building the container
`docker/build.bash`

### Running a ros image in a terminal
`docker run -it --network host ros:noetic-ros-base-focal`

# Things that need to be made generic or easily configurable or auto generated
- Build process 
    - Image and container name
    - python package versions
    - any extra dependencies e.g., cuda
    - mlflow artifact download link
- ROS msg/srv structure?
- Type conversion between model input/output and ros msgs (message_interface function)

# Notes 
- Might need to make people to use the mlflow version that's used by the tracking server?
- Probably use a config file with examples
- metaprogramming? 

# Questions
- does the user provide the ros msg structure that the model should use? 