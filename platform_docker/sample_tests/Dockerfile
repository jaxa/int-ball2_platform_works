FROM ros:noetic

RUN apt update 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get install python3-pip -y 

RUN pip3 install rospkg numpy
# numpy is used in sample programs.
# RUN pip3 install numpy==1.18.2


############################# User packages build ##############################
RUN mkdir -p /root/catkin_ws/src
COPY user_programs /root/catkin_ws/src/user_programs
RUN cd /root/catkin_ws/ && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

############################# Import shell scripts ##############################
COPY scripts /root/scripts
RUN chmod -R 744 /root/scripts
CMD ["/root/scripts/cmd.sh"]