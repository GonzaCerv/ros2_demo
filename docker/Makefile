# If you want to rebuild all images, set this to --no-cache from the commandline
DOCKER_ARGS?=
USER_ID=$(shell id -u)

.PHONY: ubuntu_bionic
ubuntu_bionic:
	docker build ${DOCKER_ARGS} --build-arg uid=$(USER_ID) -t ubuntu_bionic ubuntu_bionic

.PHONY: ubuntu_bionic_nvidia
ubuntu_bionic_nvidia:
	docker build ${DOCKER_ARGS} --build-arg uid=$(USER_ID) -t ubuntu_bionic ubuntu_bionic

.PHONY: ros_foxy
ros_foxy: ubuntu_bionic
	docker build ${DOCKER_ARGS} -t ros_foxy ros_foxy

.PHONY: ros_foxy_nvidia
ros_foxy_nvidia: ubuntu_bionic_nvidia
	docker build ${DOCKER_ARGS} -t ros_foxy_nvidia ros_foxy
