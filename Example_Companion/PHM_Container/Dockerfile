# Use an official Ubuntu 18.04 base image
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Python 3.8
RUN apt-get update && apt-get install -y \
    software-properties-common \
 && add-apt-repository ppa:deadsnakes/ppa \
 && apt-get update && apt-get install -y \
    python3.8 \
    python3.8-venv \
    python3.8-dev \
    python3-pip

# Set the working directory in the container
WORKDIR /phm_tool

# Copy the current directory contents into the container at /phm_tool
COPY . /phm_tool

# Install any needed packages specified in requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Run health check script
CMD ["python3", "./health_checks/docker_checks.py"]
