FROM ros:noetic-ros-base-focal as frozen_stage

LABEL maintainer="Leo Lin"

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update \
  && apt-get install --assume-yes --no-install-recommends \
  # fish \
  # gdb \
  # libgl1-mesa-glx \
  # libopenblas-dev \
  # ninja-build \
  python3-colcon-common-extensions \
  python3-colcon-core \
  python3-colcon-mixin \
  python3-numpy \
  python3-pip \
  wget \
  && rm -rf /var/lib/apt/lists/*

# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb \
#   && dpkg -i cuda-keyring_1.0-1_all.deb \
#   && rm cuda-keyring_1.0-1_all.deb \
#   && apt-get update \
#   && apt-get install --assume-yes --no-install-recommends \
#   cuda-command-line-tools-11-3 \
#   cuda-compiler-11-3 \
#   cuda-libraries-dev-11-3 \
#   && rm -rf /var/lib/apt/lists/*

# ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 7.0 7.5+PTX"
# ENV TORCH_NVCC_FLAGS="-Xfatbin -compress-all"
# ENV CUDA_HOME=/usr/local/cuda-11.3
# ENV MAX_JOBS=4

# RUN python3 -m pip install --no-cache-dir torch==1.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
# RUN python3 -m pip install --no-cache-dir MinkowskiEngine==0.5.4 --install-option="--force_cuda" --install-option="--blas=openblas"
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

FROM frozen_stage as base_stage

COPY install_deps.bash install_deps.bash
RUN apt-get update \
  && ./install_deps.bash \
  && rm -rf /var/lib/apt/lists/*


FROM base_stage as prod_stage
COPY install/ install/
ARG MODEL_VERSION
ENV MLFLOW_TRACKING_URI="http://ras-b2-ph.nexus.csiro.au:5000"
RUN mlflow artifacts download -u models:/tracking-quickstart/1 -d /model