# If you want to rebuild all images, set this to --no-cache from the commandline
DOCKER_ARGS?=
USER_ID=$(shell id -u)

.PHONY: ubuntu_bionic_focal
ubuntu_bionic_focal:
	docker build ${DOCKER_ARGS} --build-arg uid=$(USER_ID) -t ubuntu_bionic_focal ubuntu_bionic_focal

.PHONY: ros_foxy
ros_foxy: ubuntu_bionic_focal
	docker build ${DOCKER_ARGS} -t ros_foxy ros_foxy
