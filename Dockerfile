# ROS distribution to use
ARG ROS_DISTRO=jazzy

#####################
# Development Image #
#####################
FROM srealper/roscon25-stonefish:stonefish_base AS dev

ARG USERNAME=developer
# Install extra tools for development
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

# docker only mounts volumes at runtime, 
# so colcon cant build unmounted packages
WORKDIR /home/${USERNAME}/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
    && . install/setup.sh

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

RUN echo "PS1='\033[01;32m\]\u\033[00m\]@\033[01;31m\]\h\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc
