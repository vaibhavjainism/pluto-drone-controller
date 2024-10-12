# ROS Package to Control Pluto Drone

This project provides a **ROS package** for controlling the **Pluto Drone** via Keyboard Controls. It includes two Python nodes, `drone_command_publisher.py` and `drone_command_listener.py`, which communicate over the `drone_command` topic using the **PlutoMsg** message type from the [plutodrone ROS package](https://github.com/DronaAviation/pluto-ros-package/tree/master).

## Project Overview

- **`drone_command_publisher.py`**: 
  - Uses a `pygame` window to capture keyboard inputs for controlling the drone.
  - Publishes these commands to the `drone_command` ROS topic.
  - Supported commands:
    - **Press 'q'**: Quit the controller.
    - **w/s**: Increase/Decrease Pitch.
    - **a/d**: Increase/Decrease Roll.
    - **i/k**: Increase/Decrease Throttle.
    - **j/l**: Increase/Decrease Yaw.
    - **e**: Altitude Lock/Unlock.
    - **SpaceBar**: Arm/Disarm the drone.

- **`drone_command_listener.py`**:
  - Subscribes to the `drone_command` topic.
  - Prints the current `PlutoMsg` commands being published by the `drone_command_publisher.py`.

## Requirements

To use this package, you need to install the following dependencies:

1. **plutodrone package**: Available [here](https://github.com/DronaAviation/pluto-ros-package/tree/master). This package provides the PlutoMsg message type used for controlling the drone.
2. **pygame library**: Used to capture keyboard inputs.

### Installation Instructions

1. **Install the plutodrone ROS package**:
   Clone the repository and build it in your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/DronaAviation/pluto-ros-package.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
2. **Install the pygame library: You can install the `pygame` library using pip:**
   ```bash
   pip install pygame
### Usage

1. **Run the `drone_command_publisher.py` node**:  
   This node will open a `pygame` window to capture keyboard inputs and publish commands to the `drone_command` topic.
   
   ```bash
   rosrun drone-controller drone_command_publisher.py
   
2. **Run the `drone_command_listener.py` node:**
   This node subscribes to the `drone_command` topic and listens for published messages from the publisher node.
    
   ```bash
   rosrun drone-controller drone_command_listener.py
   
3. **Control the drone using keyboard inputs:**

  - **Press 'q'**: Quit the controller.
  - **w/s**: Increase/Decrease Pitch.
  - **a/d**: Increase/Decrease Roll.
  - **i/k**: Increase/Decrease Throttle.
  - **j/l**: Increase/Decrease Yaw.
  - **e**: Altitude Lock/Unlock.
  - **SpaceBar**: Arm/Disarm the drone.


