FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Moscow

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    locales \
    lsb-release \
    gnupg2 \
    tzdata \
    software-properties-common

RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt-get update && apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update && apt-get install -y ros-humble-desktop

RUN apt-get install -y python3-pip
RUN pip3 install -U colcon-common-extensions

RUN apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

RUN apt-get install -y ros-humble-octomap* ros-humble-navigation2 ros-humble-nav2-bringup

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

COPY ./src /home/ros2_ws/src

RUN source /opt/ros/humble/setup.bash && colcon build

CMD ["bash"]