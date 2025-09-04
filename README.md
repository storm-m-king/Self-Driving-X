# AgileX LIMO ROS 2 Simulation Environment
This repository provides a reproducible, Docker-based setup for the AgileX LIMO robot simulation using ROS 2 Humble. It is designed for students and educators to quickly launch, visualize, and interact with the LIMO robot in Gazebo and RViz, with all necessary patches and configuration included.

## Directory Structure
```
Self-Driving-X/
├── Dockerfile                   # Builds the complete ROS 2 Humble simulation environment
├── start.bat                    # Windows script to build and run the Docker container
├── start.sh                     # Mac/Linux script to build and run the Docker container
├── patches/                     # Contains patched files for simulation fixes
│   ├── ackermann_with_sensor.xacro
│   ├── empty_world.sdf
│   └── ackermann_gazebo.launch.py
├── week2_lab/                   # Source code for Week 2 lab: ROS 2 Nodes, Topics & Messages
│   ├── README.md                # Lab 2 Instructions  
│   ├── setup.py                 
│   ├── numbers_publisher.py    
│   └── filter_subscriber.py     
└── Readme.md                    # This documentation file
```


## Quick Start Guide
### Prerequisites

- Docker installed (https://docs.docker.com/get-docker/)
- VcXsrv (Windows only) for GUI support (https://sourceforge.net/projects/vcxsrv/files/vcxsrv/)
- XQuartz (Mac only) for GUI support (https://www.xquartz.org/)

## Setup Steps

1. Clone this repository:
```
git clone https://github.com/storm-m-king/Self-Driving-X.git
cd Self-Driving-X
```


2. Run the setup script:

On Linux / Mac *(NOT YET TESTED)*
```
. start.sh
```
On Windows
```
./start.bat
```

- The script detects your OS and configures Docker for GUI support.
- On Windows, start VcXsrv before running the script.
- On Mac, start XQuartz and enable “Allow connections from network clients.”
3. Inside the container, launch the simulation:
```
colcon build
source install/setup.bash
ros2 launch limo_car ackermann_gazebo.launch.py
```


- This will open Gazebo and RViz with the LIMO robot.
4. Start the robot steering GUI:
  - In another terminal, run the start script again. Then run:
```
ros2 run rqt_robot_steering rqt_robot_steering
```

- Use the GUI to control the robot in the simulation.

[Optional] If you prefer to use the keyboard instead to move the robot, then run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## What’s Included?

- Pre-installed ROS 2 Humble and all required packages
- Pre-cloned LIMO ROS 2 workspace with necessary directories
- Patched files to fix known simulation issues:
    - ackermann_with_sensor.xacro: Fixes URDF parent link error
    - empty_world.sdf: Provides a valid Gazebo world for spawning entities
    - ackermann_gazebo.launch.py: Ensures correct world file is loaded
- Automated build and environment setup via Dockerfile
- Cross-platform setup script for easy container launch

## Troubleshooting

- GUI not displaying?
    - Windows: Make sure VcXsrv is running before starting the container.
    - Mac: Start XQuartz and set DISPLAY variable if needed.
    - Linux: Ensure X11 is running and accessible.
- Build warnings about unused variables: These can be safely ignored.
- Simulation errors: All known issues from the original setup guide are patched in this repo.

## References

https://github.com/agilexrobotics/limo_ros2

https://docs.ros.org/en/humble/index.html

## About This Setup
This setup is based on the AgileX LIMO Environment Setup, which guides users through:

- Setting up Docker for ROS 2 Humble
- Installing required packages and dependencies
- Creating and building the LIMO workspace
- Fixing simulation errors with patched files
- Launching and visualizing the AgileX LIMO robot in Gazebo and RViz
- Interacting with the robot using the steering GUI
All steps are automated and reproducible, making it ideal for classroom use and self-guided learning.

For questions or issues, please open an issue in this repository. Happy simulating!












