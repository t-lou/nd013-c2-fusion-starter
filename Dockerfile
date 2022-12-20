FROM nvidia/cuda:11.3.0-cudnn8-devel-ubuntu20.04

# For creating a user
ARG UNAME=usr
ARG UID=1000
ARG GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        libgtk-3-dev \
        gettext \
        python3 \
        python3-dev \
        python3-setuptools \
        libpython3-dev \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip attrdict && \
    python3 -m pip install --no-cache-dir \
        numpy \
        opencv-python \
        open3d \
        protobuf==3.20 \
        easydict \
        pillow \
        matplotlib \
        wxpython \
        shapely \
        tqdm && \
    python3 -m pip install --no-cache-dir \
        torch==1.11.0+cu113 \
        torchvision==0.12.0+cu113 \
        torchaudio==0.11.0 \
        --extra-index-url https://download.pytorch.org/whl/cu113

# Add user and set as default
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

USER $UNAME
