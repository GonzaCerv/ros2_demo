FROM ubuntu:20.04
ARG uid
LABEL maintainer="Emiliano Borghi"

ENV USER="docker"

# Setup environment
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    apt-utils \
    bash-completion \
    build-essential \
    curl \
    gnupg2 \
    git \
    lsb-release \
    mesa-utils \
    pkg-config \
    python3-vcstools \
    python3-pip \
    software-properties-common \
    ssh \
    sudo \
    tmux \
    wget \
    vim

# Add QT4
RUN add-apt-repository ppa:rock-core/qt4 && \
    apt-get update && \
    apt-get install -y libqtcore4

RUN apt-get update && \
    apt-get install -y dirmngr && \
    apt-get upgrade -y && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Create a user with passwordless sudo
RUN adduser --gecos "Development User" --disabled-password -u ${uid} $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER root
