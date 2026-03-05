FROM ubuntu:22.04

############################# ROS2 installation ##############################
RUN apt-get update -q
RUN apt-get upgrade -yq 
RUN DEBIAN_FRONTEND=noninteractive apt-get install -yq \
            wget \
            curl \
            git \
            gnupg \
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

RUN sudo sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update -q
RUN apt-get upgrade -yq
RUN sudo apt install -yq ros-humble-ros-base
RUN sudo apt install -yq ros-dev-tools

############################# ROS1 installation ##############################
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update -q

RUN sudo apt-get install -yq \
            python3-rosdep \
            python3-rosinstall-generator \
            python3-vcstools \
            python3-vcstool

RUN sudo rosdep init

RUN mkdir -p /root/Downloads
RUN cd /root/Downloads &&\
    wget http://archive.ubuntu.com/ubuntu/pool/universe/h/hddtemp/hddtemp_0.3-beta15-53_amd64.deb && \
    sudo apt install /root/Downloads/hddtemp_0.3-beta15-53_amd64.deb

RUN cd /root/Downloads &&\
    wget https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml &&\
    sed -i '/focal: \[hddtemp\]/a \    jammy: \[hddtemp\]' base.yaml &&\
    sed -i '/base\.yaml/c yaml file:///root/Downloads/base.yaml' /etc/ros/rosdep/sources.list.d/20-default.list

RUN sudo rosdep update

RUN apt install -yq \
        cmake \
        python3-empy \
        libboost-all-dev

RUN mkdir /root/ros_catkin_ws
RUN cd /root/ros_catkin_ws &&\
    rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall &&\
    mkdir ./src &&\
    vcs import --input noetic-desktop.rosinstall ./src &&\
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

RUN cd /root/ros_catkin_ws/src &&\
    mkdir /root/Downloads/backup &&\
    mv rosconsole urdf /root/Downloads/backup/ &&\
    git clone https://github.com/dreuter/rosconsole.git &&\
    cd rosconsole &&\
    git checkout noetic-jammy &&\
    cd /root/ros_catkin_ws/src &&\
    git clone https://github.com/dreuter/urdf.git &&\
    cd urdf &&\
    git checkout set-cxx-version &&\
    cd /root/ros_catkin_ws/src

RUN cd /root/ros_catkin_ws &&\
    ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release

############################# ros1_bridge installation ##############################
# RUN sudo apt install -yq ros-humble-ros1-bridge
RUN mkdir -p /root/ros1_bridge_ws/src
RUN cd /root/ros1_bridge_ws/src &&\
    git clone https://github.com/ros2/ros1_bridge.git
RUN cd /root/ros1_bridge_ws/ && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install --packages-skip ros1_bridge"
RUN cd /root/ros1_bridge_ws/ && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

############################# User packages build ##############################
RUN mkdir -p /root/colcon_ws/src
COPY user_programs/ros2/simple_talker_ros2/ /root/colcon_ws/src/simple_talker_ros2
RUN cd /root/colcon_ws/ && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"

RUN mkdir -p /root/catkin_ws/src
COPY user_programs/ros1/simple_listener/ /root/catkin_ws/src/simple_listener
RUN cd /root/catkin_ws/ && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

############################# Import shell scripts ##############################
COPY scripts /root/scripts
RUN chmod -R 744 /root/scripts
CMD ["/root/scripts/cmd.sh"]

