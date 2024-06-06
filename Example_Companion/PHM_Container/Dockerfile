# Use an official Ubuntu 18.04 base image
FROM ubuntu:18.04

# Avoid user interaction with tzdata
#ENV DEBIAN_FRONTEND=noninteractive

# Install Python 3.8 and utilities
RUN apt-get update && \
    apt-get install -y software-properties-common iputils-ping curl && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8 python3.8-venv python3.8-dev python3-pip

# Install Docker
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add - && \
    add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" && \
    apt-get update && \
    apt-get install -y docker-ce docker-ce-cli containerd.io

# Set the working directory in the container
WORKDIR /phm_tool

# Copy the current directory contents into the container at /phm_tool
COPY . /phm_tool

# Install any needed packages specified in requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Ensure Python uses Docker commands
#ENV PATH="/usr/local/bin:${PATH}"

# Run health check script
CMD ["python3", "tests.py"]
