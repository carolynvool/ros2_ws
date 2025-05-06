# Base image: ROS 2 Humble desktop
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    sudo \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    build-essential \
    git \
    gedit \
    python3-pip && \
    pip3 install setuptools==58.2.0 && \
    pip3 install transforms3d && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a non-root user named 'ros'
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc && \
    echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

CMD ["bash"]

