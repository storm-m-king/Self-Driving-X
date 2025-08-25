FROM osrf/ros:humble-desktop-full

# Install required packages
RUN apt-get update && \
    apt-get install -qq -y --no-install-recommends \
        ros-humble-gazebo-* \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rqt-robot-steering \
        locales \
        git \
        nano \
        build-essential \
        cmake \
        libusb-1.0-0-dev \
        pkg-config \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \    
        libusb-1.0-0 \
        udev \
        apt-transport-https \
        ca-certificates \
        curl \
        swig \
        software-properties-common \
        python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11

# Create workspace and clone limo_ros2 repo
RUN mkdir -p /limo_ws/src && \
    cd /limo_ws/src && \
    git clone https://github.com/agilexrobotics/limo_ros2.git && \
    mkdir -p /limo_ws/src/limo_ros2/limo_car/log \
              /limo_ws/src/limo_ros2/limo_car/src \
              /limo_ws/src/limo_ros2/limo_car/worlds

WORKDIR /limo_ws

# Clone the YDLidar-SDK 
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git &&\
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build &&\
    cmake ..&&\
    make &&\
    make install &&\
    cd .. &&\
    pip install . &&\
    cd .. && rm -r YDLidar-SDK 

# Copy patch files from local repo into the image
COPY /patches/ackermann_with_sensor.xacro /limo_ws/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro
COPY /patches/empty_world.sdf /limo_ws/src/limo_ros2/limo_car/worlds/empty_world.sdf
COPY /patches/ackermann_gazebo.launch.py /limo_ws/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py

# Set up ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
CMD ["/bin/bash"]