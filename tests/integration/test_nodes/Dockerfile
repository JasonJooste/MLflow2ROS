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

COPY src src

# build the node
RUN . /opt/ros/noetic/setup.sh && \
    colcon build --event-handlers console_cohesion+ status-

# make entrypoint scripts
RUN echo 'source "/opt/ros/noetic/setup.bash"' >> /root/.bashrc \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /root/.bashrc \
  && echo '  source /workspace/install/setup.bash' >> /root/.bashrc \
  && echo 'fi' >> /root/.bashrc \
  && echo '#!/usr/bin/env bash' > /entrypoint.bash \
  && echo 'source /opt/ros/noetic/setup.bash' >> /entrypoint.bash \
  && echo 'if [ -e /workspace/install/setup.bash ]; then' >> /entrypoint.bash \
  && echo '  source /workspace/install/setup.bash' >> /entrypoint.bash \
  && echo 'fi' >> /entrypoint.bash \
  && echo 'exec "$@"' >> /entrypoint.bash \
  && chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
CMD ["/bin/bash"]
