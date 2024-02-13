## Overview
This repository contains the source code of the ROS node generator for MLFLow models. See below to get started. 

### Installing dependencies
1. `pip install virtualenv` (if you don't already have virtualenv installed)
2. `virtualenv venv` to create your new environment (called 'venv' here)
3. `source venv/bin/activate` to enter the virtual environment
4. `pip install -r requirements.txt` to install the requirements in the current environment

### Using the CLI
Run `./app.py --help` to get started

### Running tests
Unit test: `python -m pytest`

### Running containers with GPU
Install nvidia container toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html 
Run `docker run -it --network host --gpus all <container name>`
