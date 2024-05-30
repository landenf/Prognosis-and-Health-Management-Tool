# Use an official ROS runtime as a parent image
FROM ros:melodic-ros-core

# Set the working directory
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any needed packages specified in requirements.txt
RUN apt-get update && apt-get install -y python3-pip && pip3 install --no-cache-dir -r requirements.txt

# Copy the ROS entrypoint script
COPY ros_entrypoint.sh /

# Make the script executable
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Run app.py when the container launches
CMD ["python3", "app.py"]
