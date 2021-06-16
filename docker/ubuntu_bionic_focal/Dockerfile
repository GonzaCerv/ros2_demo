FROM ubuntu:20.04 as base_image
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

####################################################
# Create a layer and install yaml-cpp
FROM base_image as yaml_cpp
USER root
RUN apt-get update && apt-get install -y \
  g++ git autogen autoconf build-essential catkin cmake graphviz \
  libboost-dev libboost-test-dev libgtest-dev libtool \
  python3-sip-dev doxygen python3-sphinx pkg-config \
  python3-sphinx-rtd-theme
RUN git clone https://github.com/jbeder/yaml-cpp.git
RUN cd yaml-cpp/ && mkdir build && cd build && cmake .. && make && make DESTDIR=/tmp/ install

####################################################
# Layer for environment variables and basic packages
FROM base_image as base_image_with_extra_packages

# Copy seriallib from seriallib layer into the new image.
COPY --from=yaml_cpp /tmp/usr/local/ /usr/local/