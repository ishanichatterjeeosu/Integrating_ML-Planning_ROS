container_name ?= testbed-dev
image_name ?= testbed
tag_name ?= latest
username ?= developer

image_tag = $(image_name):$(tag_name)

default:
	@echo "Please use make with a valid command: build_image, build_container, remove_container, start, stop, attach"

image:
	docker build -t $(image_tag) .
	touch /tmp/.docker.xauth # Create /tmp/.docker.xauth if it does not exist

container:
	docker container run -it \
	-v $(shell pwd)/../..:/home/$(username)/catkin_ws/src \
	-p 8888:8888 \
	--name $(container_name) \
	--gpus=all \
	--env='DISPLAY' \
	--env='QT_X11_NO_MITSHM=1' \
	--volume='$(HOME)/.Xauthority:/$(username)/.Xauthority:rw' \
	--volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \
	-v "/dev/input:/dev/input" \
	-e 'TERM=xterm-256color' \
    -v "/etc/localtime:/etc/localtime:ro" \
	--privileged \
	--security-opt seccomp=unconfined \
	-u $(username):$(username) \
	$(image_tag) \
	bash

start:
	docker container start $(container_name)

stop:
	docker container stop $(container_name)

ssh:
	docker exec -it $(container_name) bash

attach:
	docker container attach $(container_name)

remove_container:
	@echo -n "Are you sure? [y/N] " && read ans && [ $${ans:-N} = y ]
	docker container rm $(container_name)
