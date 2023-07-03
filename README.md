# traffic_light_bridge
This package provides a simple implementation to send traffic light status in CARLA to Autoware Universe

https://github.com/Lamfurst/traffic_light_bridge/assets/66519297/69fc6148-5520-4504-83d6-701ecd6c3acc

## Prerequisites

Before using this package, ensure that you have the following prerequisites installed:

- ROS2 (Robot Operating System)
- CARLA simulator
- AutoWare Universe
- Follow Dr. Hatem's instructions to install the [Autoware-CARLA bridge and open planner](https://www.youtube.com/watch?v=EFH-vVxn180)
- tmux
- Vector map with stop sign and traffic signal information

## Installation

1. Follow the [doc](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to create a ROS2 workspace
2. Clone this repository into your src folder under your workspace
    ```shell
    cd <your_workspace>/src
    ```
3. Build your workspace using `colcon build --symlink-install`

## Configuration
1. Change the paths in launch_script.sh to your own paths of Carla, autoware folder, current workspace
2. Find the mapping relationship between signal ID in carla and signal ID in vector map.
3. Modify the mapping relationship in tl_map.yaml file under config folder.
4. `colcon` build again

## Usage
1. Launch Carla and Autoware
2. Launch the traffic_light_bridge by running launch_script.sh
    ```shell
    ./launch_script.sh
    ```
