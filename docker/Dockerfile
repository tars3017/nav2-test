FROM ros:humble-ros-base-jammy

ARG USERNAME=user
ARG USERID=1000
ARG USERGID=${USERID}
ARG SHELL=bash
ARG ROS_DISTRO=humble

LABEL org.opencontainers.image.authors="tars3017"
LABEL shell="${SHELL}"

ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}

# Install packages
RUN apt-get update && \
    apt-get install -y \
    sudo \
    vim \
    curl \
    wget \
    tmux \
    htop \
    git \
    git-extras \
    gnupg2 \
    net-tools \
    locales 

# Setup ros2
# RUN apt-get install -y \
#     software-properties-common \
#     ros-dev-tools && \ 
#     add-apt-repository universe && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# RUN rm /etc/apt/sources.list.d/ros2.list

# Setup nav2
COPY ./src /root/nav2-ws/src
# COPY ./install /hom/user/nav2-ws/install
# COPY ./log /home/user/nav2-ws/log
# COPY ./build /home/user/nav2-ws/build
WORKDIR /root/nav2-ws
RUN apt-get update && \
    rosdep install -y --from-paths ./src --ignore-src
# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --parallel-workers 2 --symlink-install"

RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
#     ros-${ROS_DISTRO}-navigation2 \
#     ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3-gazebo

# Set Language
RUN echo "en_US.UTF-8 UTF-8" > /etc/local.gen && locale-gen en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set timezone (Taipei)
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

# Add new user (Passwordless with sudo)
# RUN groupadd --gid ${USERGID} ${USERNAME} \
#     && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
#     # passwordless
#     && apt-get update \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && rm -rf /var/lib/apt/lists/*

# USER ${USERNAME}

# Clean apt-cache
RUN sudo apt autoremove -y && \
    sudo apt clean -y

# Default command
SHELL ["/bin/bash", "-c"]
CMD ["bash"]