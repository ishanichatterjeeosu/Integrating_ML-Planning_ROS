# Ubuntu 20.04 with nvidia cuda support
ARG BASEIMG=nvidia/cuda:11.5.0-runtime-ubuntu20.04
FROM $BASEIMG

# Set ROS distribution
ARG DIST=noetic

# Set Gazebo verison
ARG GAZ=gazebo11
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles


# Tools useful during development.
RUN apt update \
 && apt install -y \
        build-essential \
        cppcheck \
        curl \
        cmake \
        lsb-release \
        gdb \
        git \
        python3-dbg \
        python3-pip \
        python3-venv \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
        libeigen3-dev \
        pkg-config \
        protobuf-compiler \
        tzdata \
        tmux \
        htop \
 && apt clean

# Get ROS noetic and Gazebo 11.
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt update \
 && apt install -y \
    python3-rosdep \
    python3-catkin-tools \
    qtbase5-dev \
    libgles2-mesa-dev \
    ros-${DIST}-desktop-full \
    ros-${DIST}-velodyne-gazebo-plugins \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-robot-plugins \
    ros-${DIST}-rqt-common-plugins \
    ros-${DIST}-joy \
    ros-${DIST}-teleop-twist-joy \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-geographic-info \
    ros-${DIST}-move-base \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-xacro \
    ros-${DIST}-moveit\
    ros-${DIST}-graph-msgs\
    ros-${DIST}-rviz-visual-tools\
    libqt5x11extras5-dev\
 && rosdep init \
 && apt clean

# # Dependencies for MoveIt! - Only used if building MoveIt! from source
# RUN apt update \
#  && apt install -y \
#         ccache \
#         python-wstool \
#         python-catkin-tools \
#         clang-format-10 \
#         python-catkin-pkg \
#  && apt clean 


# RUN rosdep update

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Commands below run as the developer user.
USER $USER:$GROUP
# When running a container start in the developer's home folder.
WORKDIR /home/$USER

RUN mkdir -p /home/$USER/catkin_ws/src
WORKDIR /home/$USER/catkin_ws

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
# Customize your image here.