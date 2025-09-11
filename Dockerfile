# ROS distribution to use
ARG ROS_DISTRO=jazzy

##############
# Base Image #
##############
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
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
# WORKDIR /home/${USERNAME}

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
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# RUN apt-get update && apt-get install ros-noetic-controller-manager -y

COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM base AS dev

# # Install extra tools for development
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb gdbserver nano iputils-ping byobu 

WORKDIR /home/${USERNAME}
USER ${USERNAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# export nvidia env variables to use it in multi gpu systems
RUN echo "export __NV_PRIME_RENDER_OFFLOAD=1" >> ~/.bashrc && \
    echo "export __GLX_VENDOR_LIBRARY_NAME=nvidia" >> ~/.bashrc && \
    echo "export DRI_PRIME=1" >> ~/.bashrc

RUN source ~/.bashrc

WORKDIR /home/${USERNAME}/ros2_ws
COPY ./auv_stonefish /home/${USERNAME}/ros2_ws/src/auv_stonefish
# RUN colcon build

# # Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

RUN echo "PS1='\e[01;32m\u\e[0m@\e[01;31m\h\e[0m:\e[01;34m\w\e[0m\$ '" >> /home/${USERNAME}/.bashrc
