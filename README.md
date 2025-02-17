# MentorPi Ackermann Robot Simulator & RViz Joint Visualizer (ROS 2)

## Overview
A simple ROS 2 package for simulating an Ackermann-steered robot in Gazebo and visualizing its joints in RViz.  
*Note*: We currently use a Diff Drive plugin with a converter due to limited Ackermann support in ROS 2’s Gazebo transport layer.

## Installation
1. **Clone this repo** into your ROS 2 workspace (e.g., `~/ros2_ws/src`).
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ViktorNfa/mentorpia1_simulator
   ```

2. **Build the workspace**

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. **Source the setup file**:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage
1. **Launch the Simulation**
    ```bash
    ros2 launch mentorpia1_simulator mentorpia1_simulator.launch.py
    ```
    This starts Gazebo with the robot model and the Diff Drive converter. You can publish Ackermann steering commands as:
    ```bash
    ros2 topic pub /cmd_ackermann ackermann_msgs/msg/AckermannDrive "{steering_angle: 0.3, speed: 1.0}"
    ```

2. **Launch RViz**
    ```bash
    ros2 launch mentorpia1_simulator rviz.launch.py
    ```
    This opens RViz, displaying the robot model and its joints.

## Troubleshooting
Ackermann steering is not supported by the current Gazebo transport layer in ROS 2. We are using a Diff Drive plugin with a converter. Future versions will restore true Ackermann steering since an Ackermann plugin is already defined in the SDF.

## Contributing
Feel free to fork, develop, and submit a Pull Request for any improvements.

## License
Licensed under the MIT License.