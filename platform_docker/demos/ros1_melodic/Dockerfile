FROM ros:melodic

RUN apt update 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get install python-pip python3-pip -y 
RUN pip3 install --upgrade pip

RUN pip3 install rospkg numpy
# numpy is used in sample programs.
# RUN pip3 install numpy==1.18.2

RUN apt install -y ros-melodic-geometry2

############################# User packages build ##############################
RUN mkdir -p /root/catkin_ws/src
COPY user_programs/ros1/simple_talker/ /root/catkin_ws/src/simple_talker
RUN cd /root/catkin_ws/ && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make"

############################# Import shell scripts ##############################
COPY scripts /root/scripts
RUN chmod -R 744 /root/scripts
CMD ["/root/scripts/cmd.sh"]
