# ROS Foxy

## Step 1: Install Docker

[Install docker](https://docs.docker.com/engine/installation/linux/ubuntu/) and [configure after postintalling it](https://docs.docker.com/install/linux/linux-postinstall/).

To run docker without super user:

        ```bash
        $ sudo groupadd docker
        $ sudo gpasswd -a ${USER} docker
        $ sudo service docker restart
        ```

## Step 2: Use NVIDIA acceleration

### Install [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker/tree/v2.6.0)

#### Prerequisites

1. GNU/Linux x86_64 with kernel version > 3.10
2. Docker >= 19.03
3. NVIDIA GPU with Architecture > Kepler (or compute capability 3.0)
4. NVIDIA drivers >= 418.81.07 (Note that older driver releases or branches are unsupported.)

#### Removing nvidia-docker 1.0

Version 1.0 of the nvidia-docker package must be cleanly removed before continuing.
You must stop and remove all containers started with nvidia-docker 1.0.

        ```bash
        $ docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
        $ sudo apt-get purge nvidia-docker
        ```

#### Installing NVIDIA Container Toolkit

Follow steps in [Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
      ```bash
      # Install docker if you don't have it.
      $ curl <https://get.docker.com> | sh && sudo systemctl --now enable docker

      # Setup the stable repository and the GPG key.
      $ distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
            && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
            && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

      # Update and install 
      sudo apt-get update
      sudo apt-get install -y nvidia-docker2

      # Restart docker
      sudo systemctl restart docker

      # Test if it is working 
      sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
      ```

## Step 3: Creating the container

This repository contains the Dockerfile. Move into the directory containing the file and type

The command below will **create** the container from the base image if it doesn't exist and log you in.

      ```bash
      $ ./build
      ```

The build script will automatically detect whether you are running with a nvidia setup or not. However, it's also possible to force the build script to build for a nvidia setup.

      ```bash
      $ ./build --force-nvidia
      ```

## Step 4: Start the container

To run the container, you can use the run script:

      ```bash
      $ ./run
      ```

Every time you launch the Docker container, you'll need to compile the workspace and source:

      ```bash
      $ catkin_make -DCMAKE_BUILD_TYPE=Release -j4
      $ source devel/setup.bash
      ```

# References

This Repo was forked from [RoboticaUtnFrba](https://github.com/RoboticaUtnFrba/create_autonomy) repo. Special thanks to @eborghi10.

* <http://wiki.ros.org/docker/Tutorials/Docker>
* <http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration>
* <http://wiki.ros.org/docker/Tutorials/GUI>
