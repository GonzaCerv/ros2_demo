FROM ubuntu_bionic
LABEL maintainer="Emiliano Borghi"

ENV ROS_DISTRO="foxy"
ENV USER="docker"

USER root

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list

# Install bootstrap tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-pip \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Initialize rosdep
RUN rosdep init

RUN pip3 install --upgrade pip

# Install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# Automatically source ROS workspace
ENV WS_DIR=/home/${USER}/ws
RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc

# Workspace
USER $USER
RUN mkdir -p ${WS_DIR}/src/

WORKDIR ${WS_DIR}
RUN rosdep update
