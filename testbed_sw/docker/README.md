# Docker Support

We provide a DockerFile for setting up an image running ROS Noetic with MoveIt! installed to install/run our packages.

Note that these files are written to create a *development* environment rather than for *deployment*. The resulting image will mount this repository inside a Catkin Workspace.

For deployment, we will want to refactor this to the minimum dependencies.

## Docker Installation

1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
2. Do the [Post-Install Docker instructions](https://docs.docker.com/engine/install/linux-postinstall/)
3. If using an Nvidia card, install the [Nvidia docker container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

2. Restart your computer for the changes to take effect.

## Use the Makefile to execute commands

We have included a Makefile for convenience in building an image, creating a container, and launch/attaching to the image.
To build the image, run:
```bash
make image
```

To create and start the container, run:
``` bash
make container
```

To start an already built container, run:
``` bash
make start
```
To ssh into running containers (thus creating a new bash window into the container), run:
``` bash
make ssh
```

To attach to a running (started) container, run:
``` bash
make attach
```
note that attached to a running container will link to the container's tty, thus running attach multiple times will just open multiple views into the same tty interface.

## Installation
1. This container assumes you have the following file structure. Please ensure your folder structure matches.
```
<testbed_workspace> (Folder you cloned this repository into)
|_ testbed_sw (this repository)
  |_ docker
    |_ README.md (this file)
  |_ testbed_description
|_ <any other packages/folders/files you would like access to inside the container>
```
Note that all folders/files inside of `<testbed_workspace>` (the folder in which you cloned this repository) will be mounted into the container at `~/catkin_ws/src`.

2. Run `make image` to create the Docker image.
3. Run `make container` to create the Docker container from the image.
4. Exit out of the container (to ensure that the container is persistent through restarts.)
5. Run `make start` to start the container.
6. Run `make ssh` to access the container.
7. Follow the instructions in [First Time Setup](#First-Time-Setup) to setup the catkin workspace.


Once you have the container set up, all you have to do is:
1. Run `make start` to start the container
2. Run `make ssh` in as many windows as desired to have multiple views into the same container

## First Time Setup

After dropping into the Docker container, the first thing you will want to do is update rosdep, install dependencies, and build the catkin workspace:
``` bash
cd ~/catkin_ws/src
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
cd ..
catkin build
```
Don't forget to source catkin environment after building.
```bash
source ~/catkin_ws/devel/setup.bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```
