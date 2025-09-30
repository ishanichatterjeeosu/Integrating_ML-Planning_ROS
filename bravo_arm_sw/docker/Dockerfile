FROM ros:noetic-robot

ARG WORK_DIR=/catkin_ws
WORKDIR ${WORK_DIR}

# for apt to be noninteractive
ARG DEBIAN_FRONTEND=noninteractive
ARG DEBCONF_NONINTERACTIVE_SEEN=true

# install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# clone all packages
RUN mkdir src && \
    echo "test" && \
    cd src && \
    git clone https://gitlab.com/rsa-manipulation-testbed/bravo_arm_sw && \
    cd .. && \
    vcs import < src/bravo_arm_sw/bravo_arm_sw.repos

# install dependencies and build
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# build
RUN . /opt/ros/noetic/setup.sh && \
    catkin build
