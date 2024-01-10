FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update \
    && apt-get -y install python3-pip \
    && apt-get -y install git \
    && rm -rf /var/lib/apt/lists/*

COPY requirements_dev.txt requirements_dev.txt

RUN pip install --upgrade numpy \
    && pip install --upgrade python-dateutil \
    && pip install -r requirements_dev.txt

    