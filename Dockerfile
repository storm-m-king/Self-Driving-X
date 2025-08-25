FROM osrf/ros:humble-desktop-full

# Install required packages
RUN apt-get update && \
    apt-get install -y \
        software-properties-common \
        ros-humble-gazebo-* \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rqt-robot-steering \
        locales \
        git \
        nano \
        python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV GAZEBO_RESOURCE_PATH /usr/share/gazebo-11

# Create workspace and clone limo_ros2 repo
RUN mkdir -p /limo_ws/src && \
    cd /limo_ws/src && \
    git clone https://github.com/agilexrobotics/limo_ros2.git && \
    mkdir -p /limo_ws/src/limo_ros2/limo_car/log \
              /limo_ws/src/limo_ros2/limo_car/src \
              /limo_ws/src/limo_ros2/limo_car/worlds

# Copy patch files from local repo into the image
COPY Self-Driving-X/patches/ackermann_with_sensor.xacro /limo_ws/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro
COPY Self-Driving-X/patches/empty_world.sdf /limo_ws/src/limo_ros2/limo_car/worlds/empty_world.sdf
COPY Self-Driving-X/patches/ackermann_gazebo.launch.py /limo_ws/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py

# Set up ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN . /opt/ros/humble/setup.bash && colcon build

WORKDIR /limo_ws

CMD ["/bin/bash"]