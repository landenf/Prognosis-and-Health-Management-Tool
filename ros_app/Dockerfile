# syntax=docker/dockerfile:1
FROM ros:noetic-robot

# Create the image file structure
RUN mkdir -p /home/catkin_ws/src

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        netbase \
        python3 \
        python3-pip \
        git \
        nano \
        vim \
        htop \
        python3-lxml \
        python3-catkin-tools \
        python3-rosinstall-generator \
    && rm -rf /var/lib/apt/lists/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make

HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 CMD echo "OK" || exit 0

COPY ros.sh /
RUN ls -la /
RUN ["chmod", "+x", "/ros.sh"]
ENTRYPOINT ["/ros.sh"]
CMD ["bash"]