# Use an official Ubuntu 18.04 base image
FROM ubuntu:18.04

# Install Python 3.8 and pip
RUN apt-get update && \
    apt-get install -y software-properties-common iputils-ping curl && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8 python3-pip

# Install lsusb (part of usbutils) and lsblk (part of util-linux) (lspci)
RUN apt-get update && \
    apt-get install -y usbutils util-linux pciutils && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install pymavlink

# Set the working directory in the container
WORKDIR /src

# Install Docker Python SDK
RUN pip3 install --no-cache-dir docker

# Copy the Python script into the container
COPY . /src

# Run the Python script
CMD ["sh", "-c", "python3 ./main.py && tail -f /dev/null"]
