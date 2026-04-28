FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker

ARG USERNAME=dockeruser
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt update && apt upgrade -y && apt install -y \
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

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p /tmp/runtime-docker && chmod 700 /tmp/runtime-docker
RUN pip3 install --no-cache-dir kiss-slam opencv-python
WORKDIR /workspace

CMD ["bash"]

