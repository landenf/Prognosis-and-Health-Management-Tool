# Use an official ROS base image
FROM osrf/ros:melodic-desktop-bionic

# Install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python-catkin-tools \
    python3-pip \  
    ros-melodic-rosbridge-server \
    python3-yaml \  
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install PyYAML rospkg requests

# Create and initialize the catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
RUN bash -c ". /opt/ros/melodic/setup.bash; catkin_init_workspace $CATKIN_WS/src"

# Copy your catkin workspace into the Docker image
COPY ./catkin_ws/src $CATKIN_WS/src

# Build the catkin package from the workspace root
RUN bash -c ". /opt/ros/melodic/setup.bash && cd $CATKIN_WS && catkin_make"

# Source both the ROS environment and the workspace setup script upon container startup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

WORKDIR $CATKIN_WS
CMD /bin/bash -c "cd src && ls"

HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
  CMD ls || exit 1

CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && source $CATKIN_WS/devel/setup.bash && roslaunch health_check_pkg nodes.launch"]
