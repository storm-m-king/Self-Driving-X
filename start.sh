#!/bin/bash

IMAGE="osrf/ros:humble-desktop-full"
CONTAINER="limo_dev"

# Check if a container with the same name already exists
if [ "$(docker ps -a -q -f name=^/${CONTAINER}$)" ]; then
    echo -e "\e[33mWARNING: A container named '$CONTAINER' already exists.\e[0m"
    echo -e "\e[33mIf you want to remove it and start fresh, run:\e[0m"
    echo -e "\e[33m    docker rm -f $CONTAINER\e[0m"
    echo -e "\e[33mConnecting to existing container...\e[0m"
    docker exec -it $CONTAINER /bin/bash
    return 0
fi

# Build the Docker image from the local Dockerfile
echo -e "\033[32mBuilding the docker image '$IMAGE' with name '$CONTAINER'\033[0m"
docker build -t $IMAGE .

# Detect OS and set Docker run options
OS=$(uname -s)

if [[ "$OS" == "Linux" ]]; then
    # For native Linux with X11
    DISPLAY_VAR=${DISPLAY:-:0}
    XSOCK="/tmp/.X11-unix"
    echo "Detected Linux. Make sure X11 is running."
    docker run -it --name $CONTAINER --privileged \
        -e DISPLAY=$DISPLAY_VAR \
        -e QT_X11_NO_MITSHM=1 \
        -v $XSOCK:$XSOCK \
        $IMAGE

elif [[ "$OS" == "Darwin" ]]; then
    # For MacOS with XQuartz
    echo "Detected MacOS. Make sure XQuartz is running and 'Allow connections from network clients' is enabled."
    IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    DISPLAY_VAR="${IP}:0"
    XSOCK="/tmp/.X11-unix"
    docker run -it --name $CONTAINER --privileged \
        -e DISPLAY=$DISPLAY_VAR \
        -e QT_X11_NO_MITSHM=1 \
        -v $XSOCK:$XSOCK \
        $IMAGE

elif grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null ; then
    # For Windows 11 with WSLg
    echo "Detected Windows/WSLg. Make sure VcXsrv is running."
    docker run -it --name $CONTAINER --privileged \
        -e DISPLAY=host.docker.internal:0.0 \
        -e QT_X11_NO_MITSHM=1 \
        -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix \
        -v /run/desktop/mnt/host/wslg:/mnt/wslg \
        $IMAGE
else
    echo "Unknown OS. Please start the container manually with appropriate DISPLAY and volume settings."
    exit 1
fi