FROM ros:noetic-ros-base-focal as frozen_stage

LABEL maintainer="Leo Lin"

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update \
  && apt-get install --assume-yes --no-install-recommends \
  python3-colcon-common-extensions \
  python3-colcon-core \
  python3-colcon-mixin \
  python3-numpy \
  python3-pip \
  wget \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir numpy==1.23.4 open3d==0.13.0 python-dateutil==2.8.2
RUN python3 -m pip install --no-cache-dir mlflow-skinny==2.3.1


ENV COLCON_HOME=/workspace/.colcon
ENV SHELL=/bin/bash

RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /root/.bashrc \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /root/.bashrc \
  && echo '  source /workspace/install/setup.bash' >> /root/.bashrc \
  && echo 'fi' >> /root/.bashrc \
  && echo '#!/usr/bin/env bash' > /entrypoint.bash \
  && echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /entrypoint.bash \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /entrypoint.bash \
  && echo '  source /workspace/install/setup.bash' >> /entrypoint.bash \
  && echo 'fi' >> /entrypoint.bash \
  && echo 'exec "$@"' >> /entrypoint.bash \
  && chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
CMD ["/bin/bash"]

COPY /../src src/
ENV MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"
RUN mlflow artifacts download -u models:/tracking-quickstart/1 -d /model
