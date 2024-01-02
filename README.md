## Overview
This repository contains the source code of the ROS node generator for MLFLow models. See below to get started. 

### Installing dependencies
1. `pip install virtualenv` (if you don't already have virtualenv installed)
2. `virtualenv venv` to create your new environment (called 'venv' here)
3. `source venv/bin/activate` to enter the virtual environment
4. `pip install -r requirements.txt` to install the requirements in the current environment

### Changing the model and docker image attributes
Modify `generic/config.cfg`

### Running the container
Run `generic/run.bash`

### Building the container
Run `generic/build.bash`

### Running a ros image in a terminal
Run `docker run -it --network host ros:noetic-ros-base-focal`
