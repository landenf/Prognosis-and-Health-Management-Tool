# Use an official ROS runtime as a parent image
FROM ros:melodic-ros-core

# Set the working directory
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install the ping utility and other necessary packages
RUN apt-get update && \
    apt-get install -y iputils-ping python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Copy the ROS entrypoint script
COPY ros_entrypoint.sh /

# Make the script executable
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
#ENTRYPOINT ["/ros_entrypoint.sh"]

# Run app.py and keep the container alive
CMD ["sh", "-c", "python3 ./app.py && tail -f /dev/null"]
