# Use an official ROS base image
FROM ros:melodic-ros-core

# Install Python 3.8 and other dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8

# Install additional ROS dependencies and tools
RUN apt-get install -y ros-melodic-ros-base ros-melodic-rosbridge-server

# Setup the environment
ENV ROS_WS=/root/catkin_ws

# Create a catkin workspace
RUN mkdir -p $ROS_WS/src && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; cd $ROS_WS; catkin_make"

# Set the working directory to the catkin workspace
WORKDIR $ROS_WS/src

# Copy your ROS package into the container
COPY your_ros_package $ROS_WS/src/your_ros_package

# Build your ROS package
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd $ROS_WS; catkin_make"

# Source the workspace setup script on each launch
RUN echo "source $ROS_WS/devel/setup.bash" >> ~/.bashrc

# Set the default command to launch the ROS nodes
CMD ["sh", "-c", "source $ROS_WS/devel/setup.bash && \
                 rosrun your_package your_node1 && \
                 rosrun your_package your_node2 && \
                 tail -f /dev/null"]
