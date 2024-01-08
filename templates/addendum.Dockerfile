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

# Install rospkg into mlflow virtualenv or conda environment
RUN echo 'if [ -e /miniconda/bin/activate ]; then' >> /install_rospkg.bash \
  && echo '  source /miniconda/bin/activate custom_env' >> /install_rospkg.bash \
  && echo 'fi' >>/install_rospkg.bash \
  && echo 'if [ -e /opt/activate ]; then' >> /install_rospkg.bash \
  && echo '  source /opt/activate' >> /install_rospkg.bash \
  && echo 'fi' >>/install_rospkg.bash \
  && echo 'pip install rospkg' >>/install_rospkg.bash \
  && /bin/bash /install_rospkg.bash \
  && rm /install_rospkg.bash

WORKDIR /workspace

FROM base as dev
# Add ROS source commands to .bashrc for convenience 
RUN echo "source /opt/ros/{{ ros_distro }}/setup.bash" >> /root/.bashrc \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /root/.bashrc \
  && echo '  source /workspace/install/setup.bash' >> /root/.bashrc \
  && echo 'fi' >> /root/.bashrc

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
# source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /entrypoint.bash \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /entrypoint.bash \
  && echo '  source /workspace/install/setup.bash' >> /entrypoint.bash \
  && echo 'fi' >>/entrypoint.bash \
  # source envs created by MLFlow
  && echo 'if [ -e /miniconda/bin/activate ]; then' >> /entrypoint.bash \
  && echo '  source /miniconda/bin/activate custom_env' >> /entrypoint.bash \
  && echo 'fi' >>/entrypoint.bash \
  && echo 'if [ -e /opt/activate ]; then' >> /entrypoint.bash \
  && echo '  source /opt/activate' >> /entrypoint.bash \
  && echo 'fi' >>/entrypoint.bash \
  # launch ROS node
  && echo "rosrun {{ model_name }} {{ model_name }}" >> /entrypoint.bash

ENTRYPOINT ["/bin/bash", "/entrypoint.bash"]