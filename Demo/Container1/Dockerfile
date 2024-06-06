# Use an official Ubuntu 18.04 base image
FROM ros:melodic-ros-core

# Install Python 3.8
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8

# Set the working directory in the container
WORKDIR /src

# Copy the Python script into the container
COPY . /src

# Run the Python script
CMD ["sh", "-c", "python3 ./app.py && tail -f /dev/null"]
