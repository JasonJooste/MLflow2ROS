# Install ROS1 Noetic. Reference: http://wiki.ros.org/noetic/Installation/Ubuntu 
# To be appended to the mlflow generated dockerfile

ARG UBUNTU_DISTRO="focal"
ARG ROS_DISTRO="noetic"

# Disable any interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Set up keys
RUN apt-get install -y gnupg
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu /$UBUNTU_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'

# Installation
RUN apt-get update && \
  apt-get install -y ros-$ROS_DISTRO-ros-base

WORKDIR /workspace

# Add ROS source commands to .bashrc for convenience 
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /root/.bashrc \
  && echo '  source /workspace/install/setup.bash' >> /root/.bashrc \
  && echo 'fi' >> /root/.bashrc

FROM base as dev
RUN apt-get install --assume-yes --no-install-recommends python3-colcon-common-extensions 
RUN pip install configobj

# Build the model's ROS node and install it on the image 
FROM base as build

RUN apt-get install --assume-yes --no-install-recommends python3-colcon-common-extensions 

COPY src src

# Build ROS packages 
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --event-handlers console_cohesion+ status-

FROM base as prod

COPY --from=build /workspace/install install