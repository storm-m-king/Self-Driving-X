# Week 2 Lab: ROS 2 Nodes, Topics & Messages

## Overview

This lab introduces core ROS 2 communication concepts through hands-on Python exercises. You’ll create publisher and subscriber nodes, experiment with topics, messages, and Quality of Service (QoS) settings, and use ROS 2 introspection tools to visualize and debug your system.

## Learning Objectives

- Understand how ROS 2 nodes communicate using topics and messages.
- Create and run publisher and subscriber nodes in Python.
- Use introspection tools (`ros2 topic`, `ros2 node`, `rqt_graph`) to visualize and debug the ROS graph.
- Experiment with topic names, message types, and QoS settings.

## Prerequisites

- ROS 2 Humble installed (or Docker environment ready)
- Basic Linux command-line knowledge
- Review [Intro ROS2 Communication](../documents/Intro-ROS2-Communication.pdf)

## Background Info

### Key ROS 2 Concepts

- **Nodes**: Independent processes performing specific tasks.
- **Topics**: Named channels for asynchronous publish/subscribe communication.
- **Messages**: Strongly typed data structures exchanged between nodes.
- **QoS (Quality of Service)**: Policies that control message delivery, reliability, and history.
- **Introspection Tools**: CLI and GUI tools for debugging and visualization (e.g., `ros2 topic list`, `ros2 topic echo`, `rqt_graph`).

### Parameters and Remapping

- **Parameters**: Allow you to change node behavior without editing code. (e.g., frequency, threshold).
  - Syntax:
    ```
    ros2 run <package> <executable> --ros-args -p <param_name>:=<value>
    ```
  - Example:  
    ```
    ros2 run week2_lab numbers_publisher --ros-args -p frequency:=10.0
    ```
- **Remapping**: Allows you to change topic or node names at runtime.
  - Syntax:
    ```
    ros2 run <package> <executable> --ros-args --remap <old_name>:=<new_name>
    ```
  - Example:  
    ```
    ros2 run week2_lab numbers_publisher --ros-args --remap numbers:=/sensors/numbers
    ```


### QoS (Quality of Service) in ROS 2
- QoS defines **how messages are delivered** between publishers and subscribers. It ensures reliability, performance, and flexibility in different network conditions.
  
#### Key QoS Policies and Their Values:
1. **Reliability**

- `reliable`: Guarantees message delivery (retries if needed).
- `best_effort`: Drops messages if the network is congested.

2. **Durability**

- `volatile`: No history; late subscribers miss old messages.
- `transient_local`: Stores last message for late joiners (similar to ROS 1 latched topics).

3. **History**

- `KEEP_LAST`: Stores the last N messages (defined by depth).
- `KEEP_ALL`: Stores all messages (can use lots of memory).

4. **Depth**

- Integer value (e.g., 10): Number of messages to store when using KEEP_LAST.

5. **Deadline**

- Time period within which messages are expected (default: infinite).

6. **Lifespan**

- How long a message remains valid (default: infinite).

7. **Liveliness**

- `automatic`: Default; system checks liveliness.
- `manual_by_topic`: Publisher asserts liveliness manually.

#### Why QoS Matters

- Ensures compatibility between publishers and subscribers.
- Helps in real-time systems where message loss or delay is critical.
- Use `ros2 topic info <topic name> --verbose` to check QoS settings.

***See [Quality of Service settings](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html) for more info.***



## Lab 2 Instructions

### 1. **Workspace Setup:**
- Create a ROS2 package named `week2_lab`:
   ```bash  
    mkdir -p ~/limo_ws/src
    cd ~/limo_ws/src
    ros2 pkg create --build-type ament_python week2_lab --dependencies rclpy std_msgs
   ```

### 2. **Implement Nodes & Setup**

Add the following nodes in the week2_lab/week2_lab/ folder

_(Full code provided in this repo.)_

#### 1. Publisher Node `numbers_publisher.py`

- Publishes random integers on topic numbers.
- Frequency controlled by a parameter named frequency.
- QoS settings Reliability, Durability, History, and Depth controlled by parameters.


#### 2. Subscriber Node `filter_subscriber.py`

- Subscribes to numbers.
- Prints values above a threshold parameter named threshold.
- QoS settings Reliability, Durability, History, and Depth controlled by parameters.

#### Modify `setup.py` to include the following in `entry_points`:
```python
entry_points={
    'console_scripts': [
        "numbers_publisher = week2_lab.numbers_publisher:main",
        "filter_subscriber = week2_lab.filter_subscriber:main"
    ],
},
```

#### Directory Scaffold to aim for by end of setup:
```
limo_ws/
└── src/
    └── week2_lab/
        ├── package.xml
        ├── resource/
        ├── setup.cfg 
        ├── setup.py                   # Python package build script
        ├── test/              
        └── week2_lab/                 # Python module directory
            ├── __init__.py 
            ├── numbers_publisher.py   # Publisher node script
            └── filter_subscriber.py   # Subscriber node script
```

### 3. **Build and Source**
```bash
cd ~/limo_ws
colcon build
source install/setup.bash
```

### 4. **Run and Visualize**
- Start publisher and subscriber in separate terminals:
```bash
ros2 run week2_lab numbers_publisher
ros2 run week2_lab filter_subscriber
```

- In a third terminal, open rqt_graph:
```bash
rqt_graph
```
- Observe the nodes and topics.

### 5. **Experiments**
#### ***A. Frequency Change***
- Run publisher with different frequencies. For example:
```bash
ros2 run week2_lab numbers_publisher --ros-args -p frequency:=1.0
```
- Measure the rate:
```bash
ros2 topic hz /numbers
```
#### ***B. Topic Remapping***
- Remap the topic to give it a different name:
```bash
ros2 run week2_lab numbers_publisher --ros-args --remap numbers:=/sensors/numbers
```
- Update subscriber similarly and observe the changes via `rqt_graph`

#### ***C. Message Type Mismatch***
- Change the subscriber only to use `Float32` instead of `Int32` and rebuild. Update only the following lines in `filter_subscriber.py`:
```python
from std_msgs.msg import Float32
```
```python
self.sub = self.create_subscription(Float32, 'numbers', self._cb, qos)
```
```python
def _cb(self, msg: Float32):
```
- Observe that nodes no longer connect when starting the subscriber and publisher.
#### ***D. QoS Settings***
- Set the publisher and subscriber to incompatible QOS settings:

- Publisher: `best_effort`:
```bash
ros2 run week2_lab numbers_publisher --ros-args -p qos.reliability:=best_effort
```

- Subscriber: `reliable`:
```bash
ros2 run week2_lab filter_subscriber --ros-args -p qos.reliability:=reliable   
```

- Check QOS compatibility with the following commands:
```bash
ros2 topic info /numbers --verbose
ros2 doctor --report
```

## Deliverables
1. Screenshot of rqt_graph showing nodes and topics.
2. Output of ros2 topic hz at two different frequencies.
3. Explanation of a QoS incompatibility and how you diagnosed it.
4. Reflection: Which factor (frequency, topic name, type, QoS) surprised you most?

## Additional Practice 
- Start the publisher and subscriber. Add a second subscriber node with a different threshold and name. Observe both subscribers in `rqt_graph`.
```bash
ros2 run week2_lab filter_subscriber --ros-args –-remap __node:=filter_subscriber2 –p threshold:=70 
```

- Use rqt_plot to visualize numeric data:
```bash
ros2 run rqt_plot rqt_plot /numbers/data
```

- Record and replay data using ros2 bag.
```bash
ros2 bag record /numbers
ros2 bag play <bag_folder>
```

### Done? Check your work: [Lab2 Deliverables](../documents/Lab2-Deliverables.pdf)











