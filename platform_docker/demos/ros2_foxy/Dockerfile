FROM ubuntu:20.04

############################# ROS2 installation ##############################
RUN apt-get update -q
RUN apt-get upgrade -yq 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq \
            wget \
            curl \
            git \
            build-essential \
            vim \
            sudo \
            lsb-release \
            locales \
            bash-completion \
            glmark2 \
            tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL   en_US.UTF-8
ENV LANG     en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

RUN apt-get update -q
RUN apt-get upgrade -yq
RUN sudo apt install -yq software-properties-common
RUN sudo add-apt-repository universe

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update -q
RUN apt-get upgrade -yq
RUN sudo apt install -yq ros-foxy-ros-base python3-argcomplete
RUN sudo apt install -yq ros-dev-tools

############################# ROS1 installation ##############################
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update -q
RUN sudo apt install -yq ros-noetic-ros-base

############################# ros1_bridge installation ##############################
RUN sudo apt install -yq ros-foxy-ros1-bridge
# RUN mkdir -p /root/ros1_bridge_ws/src
# RUN cd /root/ros1_bridge_ws/src &&\
#     git clone -b foxy https://github.com/ros2/ros1_bridge.git
# RUN cd /root/ros1_bridge_ws/ && \
#     /bin/bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros/foxy/setup.bash; colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

############################# User packages build ##############################
RUN mkdir -p /root/colcon_ws/src
COPY simple_talker_ros2/ /root/colcon_ws/src/simple_talker_ros2
RUN cd /root/colcon_ws/ && \
    /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build"

RUN mkdir -p /root/catkin_ws/src
COPY simple_listener/ /root/catkin_ws/src/simple_listener
RUN cd /root/catkin_ws/ && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"


COPY cmd.sh /cmd.sh
RUN chmod 744 /cmd.sh
CMD ["/cmd.sh"]



