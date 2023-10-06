# Dobot Magician ROS 2-based multi-robot system
<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Note**
> Installation instructions can be found on the [**main**](https://github.com/jkaniuka/magician_ros2/tree/main) branch.
  
> **Warning**
> Control system that allows you to operate several robots connected to the same network/computer is a modification of the base version of the system (see the [**main**](https://github.com/jkaniuka/magician_ros2/tree/main) branch). Only modifications are described below - full documentation is available on the [**main**](https://github.com/jkaniuka/magician_ros2/tree/main) branch.


<p align="center">
<img src="https://github.com/jkaniuka/dobot_ros2/assets/80155305/a24117cb-fdd3-41b9-a24d-b1ff1022da0e"/>
</p> 
<h4 align="center">Diagram of an example configuration of a multi-robot system</h4> 

## Table of contents :clipboard:
* [System startup](#running)
* [Control system nodes](#nodes)
* [Dobot Magician Control Panel - RQT plugin](#dmcp)
* [Examples](#examples)
* [Contributing](#contributing)

<a name="running"></a>
## System startup :robot:
:warning: If you want to connect multiple robots to one computer, modify `dobot_configuration.py` file in `dobot_driver` package (example file contents can be found below). Including appropriate configuration in this file makes it possible to connect multiple Dobot Magician robots to a single computer/network. After specifying the configuration, use `colcon build` command.

```python
from dobot_driver.interface import Interface

manipulators = {"/magician1": Interface('/dev/ttyUSB0'),
                "/magician2": Interface('/dev/ttyUSB1')
                }
```


1. Connect Dobot Magician with a USB cable to the computer and then turn it on. 
2. Set the MAGICIAN_TOOL environment variable describing the robot's configuration `export MAGICIAN_TOOL=<tool_type>` (allowed values are: _none, pen, suction_cup, gripper, extended_gripper_).
3. From the root of your workspace, run `. install/setup.bash` to source your workspace.
4. Launch entire control stack with `ros2 launch dobot_bringup dobot_magician_control_system.launch.py`. You must also provide launch argument, which is the name (ROS 2 namespace) of the robot to be launched, for example `robot_name:=magician1`.

<a name="nodes"></a>
## Nodes (assuming `/magician1` namespace)
- /magician1/dobot_PTP_server
- /magician1/dobot_gripper_srv
- /magician1/dobot_homing_srv
- /magician1/dobot_state_publisher
- /magician1/dobot_suction_cup_srv
- /magician1/dobot_trajectory_validation_server


<a name="dmcp"></a>
## Dobot Magician Control Panel
Dobot Magician Control Panel is an RQT plugin that allows you to conveniently position the manipulator, visualize its state, modify motion parameters and control end effectors. In order to launch it,  run the following command:
```
ros2 run dobot_control_panel dobot_control_panel --ros-args -r __ns:=/magician1
```
:arrow_right: After opening _Dobot Magician Control Panel_, the namespace under which the robot was launched will appear on the main screen. This will help you distinguish which control panel is used to control which robot.   
:warning: When you use _Dobot Magician Control Panel_, no other robot control program can be run. Either the robot is controlled in **manual mode** or in **automatic mode**.   



<a name="examples"></a>
## Examples
Sample scripts have been included in **dobot_demos** package. Analysis of sample codes (_minimal working examples_) from this package will help you understand how to control Dobot Magician robot.  
Running sample scripts:  
```
ros2 run dobot_demos <executable_name> --ros-args -r __ns:=/magician1
```

<a name="contributing"></a>
## Contributing

#### Bug Reports & Feature Requests

Please use the [**issue tracker**](https://github.com/jkaniuka/magician_ros2/issues) to report any bugs or feature requests.




