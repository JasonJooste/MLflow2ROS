FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update \
    && apt-get -y install python3-pip \
    && apt-get -y install git \
    && apt-get install --assume-yes --no-install-recommends python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade numpy \
    && pip install --upgrade python-dateutil \
    && pip install mlflow==2.9.2

# copy tensor basic test files, the app and the templates
COPY src src

# build the node
RUN . /opt/ros/noetic/setup.sh && \
    colcon build --event-handlers console_cohesion+ status-

# colcon test entrypoint
RUN echo 'source /opt/ros/noetic/setup.bash' >> /activate_ros_env.bash \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /activate_ros_env.bash \
  && echo '  source /workspace/install/setup.bash' >> /activate_ros_env.bash \
  && echo 'fi' >> /activate_ros_env.bash

RUN echo 'source /activate_ros_env.bash' >> /entrypoint.bash \
  && echo "rosrun test_tensor_basic test_tensor_basic " >> /entrypoint.bash 

ENTRYPOINT ["/bin/bash", "/entrypoint.bash"]