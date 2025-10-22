# ROS distribution to use
ARG ROS_DISTRO=jazzy

##############
# Base to precompile Stonefish and Stonefish ros2 #
##############
FROM osrf/ros:${ROS_DISTRO}-desktop AS stonefish_base
RUN userdel -r ubuntu
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt update && apt install -y --no-install-recommends \
    git \
    locales \
    xvfb

ARG UID=1000
ARG GID=1000
ARG USERNAME=developer

# Create new user and home directory
RUN groupadd -g $GID ${USERNAME}
RUN useradd -rm -d /home/${USERNAME} -s /bin/bash -g $GID -G sudo -u $UID ${USERNAME} -p "$(openssl passwd -1 ${USERNAME})"

RUN rm /etc/apt/apt.conf.d/docker-clean &&\
    apt-get update
RUN touch /home/${USERNAME}/.sudo_as_admin_successful

RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

WORKDIR /home/${USERNAME}
#set developer as user
USER ${USERNAME}

RUN sudo apt update && sudo apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-xacro\    
    ros-${ROS_DISTRO}-controller-manager\    
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-pid-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-rqt

# stonefish dependencies
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    libglm-dev libsdl2-dev libfreetype6-dev 
# clone, build and install stonefish
RUN mkdir libs && cd libs && git clone https://github.com/patrykcieslak/stonefish.git && \
    cd stonefish && mkdir build && cd build && cmake .. && make -j4 && sudo make install

# Create an deploy colcon workspace
RUN mkdir -p /home/${USERNAME}/ros2_ws/src
RUN cd ros2_ws/src && git clone https://github.com/patrykcieslak/stonefish_ros2.git

WORKDIR /home/${USERNAME}/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
    && . install/setup.sh

COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
