### To be appended to the mlflow generated dockerfile ###
# Install ROS1 Noetic. Reference: http://wiki.ros.org/noetic/Installation/Ubuntu 

# Disable any interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Set up keys
RUN apt-get install -y gnupg
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu /{{ ubuntu_distro }} main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS
RUN apt-get update && apt-get install -y ros-{{ ros_distro }}-ros-base

# Create script that activates the mlflow conda env or virtual env, depending on what got installed
RUN echo 'if [ -e /miniconda/bin/activate ]; then' >> /activate_mlflow_env.bash \
  && echo '  source /miniconda/bin/activate custom_env' >> /activate_mlflow_env.bash \
  && echo 'fi' >>/activate_mlflow_env.bash \
  && echo 'if [ -e /opt/activate ]; then' >> /activate_mlflow_env.bash \
  && echo '  source /opt/activate' >> /activate_mlflow_env.bash \
  && echo 'fi' >>/activate_mlflow_env.bash 

# Create script that sources the ROS environment
RUN echo 'source /opt/ros/noetic/setup.bash' >> /activate_ros_env.bash \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /activate_ros_env.bash \
  && echo '  source /workspace/install/setup.bash' >> /activate_ros_env.bash \
  && echo 'fi' >> /activate_ros_env.bash

# Install rospkg into mlflow virtualenv or conda environment
RUN echo 'source /activate_mlflow_env.bash' >> /install_rospkg.bash \ 
  && echo 'pip install rospkg' >>/install_rospkg.bash \
  && /bin/bash /install_rospkg.bash \
  && rm /install_rospkg.bash

WORKDIR /workspace

FROM base as dev
# Add ROS source commands to .bashrc for convenience 
RUN echo 'source /activate_ros_env.bash' >> /root/.bashrc

RUN apt-get install --assume-yes --no-install-recommends python3-colcon-common-extensions 

# Build the model's ROS node and install it on the image 
FROM base as build
RUN apt-get install --assume-yes --no-install-recommends python3-colcon-common-extensions 
COPY {{ rospkg_dir }} src

# Build ROS packages 
RUN . /opt/ros/{{ ros_distro }}/setup.sh && \
    colcon build --event-handlers console_cohesion+ status-

FROM base as prod
COPY --from=build /workspace/install install

# create entrypoint script that runs the ROS node
RUN echo 'source /activate_ros_env.bash' >> /entrypoint.bash \
  && echo 'source /activate_mlflow_env.bash' >> /entrypoint.bash \
  # launch ROS node
  && echo "rosrun {{ model_name }} {{ model_name }}" >> /entrypoint.bash

ENTRYPOINT ["/bin/bash", "/entrypoint.bash"]