FROM ros:melodic

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install python-pip python3-pip -y && \
    pip3 install --upgrade pip

RUN pip3 install rospkg numpy
# numpy is used in sample programs.
RUN pip3 install numpy==1.18.2

COPY cmd.sh /cmd.sh
RUN chmod 744 /cmd.sh
CMD ["/cmd.sh"]
