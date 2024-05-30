# Use an official Python runtime as a parent image
FROM python:3.8-slim

# Set the working directory
WORKDIR /phm_tool

# Copy the current directory contents into the container at /phm_tool
COPY . /phm_tool

# Install any needed packages specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Setup sources.list and keys for ROS
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
 && curl -sSL "http://packages.ros.org/ros.key" | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Ensure ROS environment variables are set
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Run health check script
CMD ["python", "health_checks/health_script.py"]
