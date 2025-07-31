FROM ros:humble-perception-jammy


ENV ROS_DISTRO=humble
RUN apt update && apt install -y \
    ros-humble-xacro \
    ros-humble-backward-ros \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-pinocchio \
    git \
    nano \
    tmux \
    vim \
    bash-completion \
    udev \
    setserial


SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "sudo setserial /dev/ttyUSB0 low_latency" >> ~/.bashrc

CMD ["bash"]
