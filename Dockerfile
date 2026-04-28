FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker

RUN apt update &&
    apt upgrade && 
    apt install -y \
    sudo \
    locales \
    vim \
    git \
    nano \
    gedit \
    build-essential \
    python3 \
    python3-pip \
    python3-venv \
    libgl1 \ 
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/runtime-docker && chmod 700 /tmp/runtime-docker
RUN pip3 install --no-cache-dir kiss-slam opencv-python
WORKDIR /workspace

CMD ["bash"]
