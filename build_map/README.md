# Build Map
The `build_map` package levaraged the `hector_mapping` package provided by ROS.
## Setup
Simply install the `hector_mapping` package from ros by running
```
sudo apt-get install ros-noetic-hector-mapping
```
## Usage
* First install necessary packages. (Check out **Setup** section)
* Directly run the launch file to run the programm
```
roslaunch build_map build_map.launch
```
* Use the `key_ctrl` package to use the keyboard to control the robot to scan the room to build the map.
    - **NOTE**: you have to be **SLOW** to make the map more accurate. The recommended linear speed is <=0.5 and the recommended angular speed is <=0.45. Use `q`/`z`, `w`/`x`, and `e`/`c` to adjust the linear and angular speed. You can check out your speed from the debug console output from the simulation.
