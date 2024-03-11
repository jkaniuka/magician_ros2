# ROS 2 control stack for Dobot Magician  
<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
<img src="https://user-images.githubusercontent.com/80155305/212770939-2d7a4389-4143-4147-a50a-3453f73a1317.png" width="250" height="250"/><img src="https://user-images.githubusercontent.com/80155305/207256203-75f2607e-b40c-4a45-bf4e-de4aaffe6530.png" width="350" height="250"/><img src="https://user-images.githubusercontent.com/80155305/204082368-6eb63a16-2a22-4aed-83f8-45730a7b5d93.png" width="200" height="250"/>
</p> 

## Table of contents :clipboard:
* [Packages in the repository](#packages)
* [Installation](#installation)
* [System startup](#running)
* [Homing procedure](#homing)
* [Diagnostics](#diagnostics)
* [Published topics](#topics)
* [Motion](#motion)
* [Visualization in RViz](#visualization)
* [Dobot Magician Control Panel - RQT plugin](#dmcp)
* [End effectors](#end_effector_control)
* [Additional tools for visualization](#additional)
* [Examples](#examples)
* [Sliding rail](#rail)
* [Multi-robot system (MRS)](#mrs)
* [Video - see how the system works](#video)
* [FAQ](#faq)
* [Citing](#citing)
* [Contributing](#contributing)


<a name="packages"></a>
## Packages in the repository :open_file_folder:

  - `dobot_bringup` - launch files and parameters configuration (in _YAML_ files)
  - `dobot_control_panel` - RQT plugin to control Dobot Magician robotic arm (as well as sliding rail)
  - `dobot_demos` - a collection of sample scripts for beginners (_minimal working examples_)
  - `dobot_description` - package containing URDF description of Dobot Magician together with meshes
  - `dobot_diagnostics` - aggregation and analysis of alarm states
  - `dobot_driver` - low-level Python interface to communicate with Dobot via serial port 
  - `dobot_end_effector` - set of service servers allowing to control different kinds of end effectors
  - `dobot_homing` - tool for executing homing procedure of Dobot stepper motors
  - `dobot_kinematics` -  implementation of forward and inverse kinematics to validate trajectory feasibility
  - `dobot_motion` - package containing action server to control motion of Dobot Magician (joint interpolated motion / linear motion)
  - `dobot_msgs` -  package defining messages used by control stack
  - `dobot_state_updater` - package containing a node regularly retrieving information about the state of the robot (e.g. joint angles / TCP position)
  - `dobot_visualization_tools` - useful tools for visualization (e.g. trajectory / range) in form of RViZ markers

<a name="installation"></a>
## Installation :arrow_down:

### Requirements

This control system requires a system setup with ROS 2. It is recommended to use Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html), however using Ubuntu 20.04 with [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html) should also work.

### Install ROS 2 Humble Hawksbill
Follow the instructions from the [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). There are 3 versions of ROS 2 Humble Hawksbill to choose from: Desktop Install, ROS-Base Install and Development tools Install. Be sure to install **Desktop Install** version (`sudo apt install ros-humble-desktop`).


### Install additional modules and packages  
All necessary modules are in [requirements.txt](https://github.com/jkaniuka/magician_ros2/blob/main/requirements.txt), install using: `pip3 install -r requirements.txt`   
Packages from apt repository: `sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl`    
:warning: After installing new RQT plugins run `rqt --force-discover` to make plugins visible in RQT GUI. This issue is further described [here](https://answers.ros.org/question/338282/ros2-what-is-the-rqt-force-discover-option-meaning/).

### Create workspace for control system (build from source)
```
source /opt/ros/humble/setup.bash
mkdir -p ~/magician_ros2_control_system_ws/src
git clone https://github.com/jkaniuka/magician_ros2.git ~/magician_ros2_control_system_ws/src
cd magician_ros2_control_system_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

### Access to serial ports  
In order to communicate with the robot, access to serial ports is required. To be able to open serial ports without using `sudo`, you need to add yourself to the **dialout** group:

```bash
sudo usermod -a -G dialout <username>
# Relogin or reboot required!
```

:warning: The USB port to which Dobot Magician is connected is set by default to **/dev/ttyUSB0** - you can change it in [this file](./dobot_driver/dobot_driver/dobot_handle.py). 

<a name="running"></a>
## System startup :robot:
1. Connect Dobot Magician with a USB cable to the computer and then turn it on. 
2. Set the MAGICIAN_TOOL environment variable describing the robot's configuration `export MAGICIAN_TOOL=<tool_type>` (allowed values are: _none, pen, suction_cup, gripper, extended_gripper_).
3. From inside of the **magician_ros2_control_system_ws** directory, run `. install/setup.bash` to source your workspace.
3. Launch entire control stack with `ros2 launch dobot_bringup dobot_magician_control_system.launch.py`. 


<a name="homing"></a>
## Homing procedure
Homing should be performed as the first action after the system is started. It is necessary because an incremental encoder has been placed in the base of the manipulator, and the robot is not aware of its actual position when it is powered up. Stop all other scripts controlling the robot before starting the homing procedure.   
Homing is handled by the service server, to start it run the following command:
```
ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure
```
A _homing\_position_ parameter is assigned to the server node of the homing service, which allows you to determine the position reached by the manipulator after the procedure is completed. The following are examples of the commands for reading and setting parameters value. 
```
ros2 param get /dobot_homing_srv homing_position  
ros2 param set /dobot_homing_srv homing_position [150.0,0.0,100.0,0.0]
```

<a name="diagnostics"></a>
## Diagnostics
The system takes care of the robot's state diagnostics. Using the _Diagnostics Viewer_ RQT plugin, you can clearly visualize information about active alarm states. To start _Diagnostics Viewer_ run the following command:
```
rqt -s rqt_robot_monitor
```
If you already have an open RQT you will find this plugin in section _Plugins -> Robot Tools -> Diagnostics Viewer_. After opening the plugin, select _Alternative view_ option at the top. After double-clicking on the alarm description, a window will appear with information about its probable cause and how to delete it. Below you will find screenshots of the _Diagnostics Viewer_ plugin. 
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220293202-d1320648-719d-4e3c-a592-52e8607d3838.png" width="400" height="400"/><img src="https://user-images.githubusercontent.com/80155305/220293214-8e07c4ef-67fa-40c1-a562-7c97e81730ff.png" width="275" height="400"/>
</p> 

<a name="topics"></a>
## Published topics
- `/joint_states` (sensor_msgs/msg/JointState) - angles values in the manipulator's joints 
- `/dobot_TCP` (geometry_msgs/msg/PoseStamped) - position of the coordinate frame associated with the end of the last robot link (orientation given as a quaternion)
- `/dobot_pose_raw` (std_msgs/msg/Float64MultiArray) - position of the coordinate frame associated with the end of the last robot link (raw orientation received from Dobot, expressed in degrees)

<a name="motion"></a>
## Motion
The motion of the manipulator is handled using the ROS 2 action. In order for the manipulator to move to the desired position, the motion target must be sent to the action server. The following describes the structure of the message sent to the action server (part of _dobot\_msgs/action/PointToPoint_):
* **motion_type**:
  * 1 -> joint interpolated motion, target expressed in Cartesian coordinates 
  * 2 -> linear motion, target expressed in Cartesian coordinates 
  * 4 -> joint interpolated motion, target expressed in joint coordinates 
  * 5 -> linear motion, target expressed in joint coordinates 
* **target_pose** - desired position expressed in Cartesian coordinates [_mm_] or in joint coordinates [_degrees_]
* **velocity_ratio** (default 1.0)
* **acceleration_ratio** (default 1.0)  
  
An example of a command that allows you to send a goal to an action server can be found below (adding `--feedback` flag will cause the terminal to display the current position of the robot while it is moving):
```
ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [200.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback
```
If you want to cancel the goal, run the following command:
```
ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal
```
The parameters associated with the motion action server node allow you to determine the motion velocities and accelerations of the individual joints of the robot (`JT1_vel, JT2_vel, JT3_vel, JT4_vel, JT1_acc, JT2_acc, JT3_acc, JT4_acc`), as well as the parameters of the linear motion of the manipulator (`TCP_vel, end_tool_rot_vel, TCP_acc, end_tool_rot_acc`). 


### Goal/trajectory validation
The motion target is checked by the trajectory validation service server before execution. The trajectory validation service server checks whether the target point is in the manipulator's workspace and whether there is a solution to the inverse kinematics task.  
  
The parameters of the node implementing the trajectory validation server are: `axis_1_range`, `axis_2_range`, `axis_3_range`, `axis_4_range`.
* The first four of these allow the manipulator's working space to be limited by setting position restrictions at joints other than those resulting from the mechanical design. If you send a motion order to a point that violates the restrictions you defined, you will receive the following response from the PointToPoint action server:
```
Response: [PTP_server-1] [WARN] [1668081940.281544573] [dobot_PTP_server]:  
Goal rejected: dobot_msgs.srv.EvaluatePTPTrajectory_Response(is_valid=False,  
message='Joint limits violated')  
```


<a name="visualization"></a>
## Visualization in RViz
In Rviz, you can display one of up to 8 different robot configurations. All allowed configurations are placed in the diagram below:
![image](https://user-images.githubusercontent.com/80155305/212382287-59fe1bd2-4e2e-4e00-ac8e-f1dd359ecaee.png)  
The command that starts the visualization of the manipulator in the example configuration is as follows: 
```
ros2 launch dobot_description display.launch.py DOF:=4 tool:=extended_gripper use_camera:=true
```
If the robot is disconnected from the computer, you can start the visualization by adding the `gui:=true` argument and control the robot using [**joint_state_publisher_gui**](https://index.ros.org/p/joint_state_publisher_gui/) node. 
  
Below you will find 3 sample visualizations:
<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212384408-b57bfde4-b837-4513-8ea3-89a7bffd97af.png" width="260" height="260"/><img src="https://user-images.githubusercontent.com/80155305/212382237-fe2eadbf-19bc-4e6c-86f4-3a0e06edccf4.png" width="260" height="260"/><img src="https://user-images.githubusercontent.com/80155305/212383859-5afbccb4-dc07-4a80-a705-3a7af915c96e.png" width="260" height="260"/>
</p> 


<a name="dmcp"></a>
## Dobot Magician Control Panel
Dobot Magician Control Panel is an RQT plugin that allows you to conveniently position the manipulator, visualize its state, modify motion parameters and control end effectors. In order to launch it,  run the following command:
```
rqt -s dobot_control_panel
```
:warning: When you use _Dobot Magician Control Panel_, no other robot control program can be run. Either the robot is controlled in **manual mode** or in **automatic mode**.   
Below you will find screenshots of all the plugin screens:

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220214176-0c0844c0-447b-481c-941b-aafd72b94d5a.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/220214179-14b4493f-4dd0-4de5-89fc-d8c60e9d4d8b.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/220214183-296eac2b-a3b0-4a84-b799-145440944eef.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp
</p> 

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/220214189-8eb57aca-bed8-4b55-a3fc-25244a41e721.png" width="250" height="230"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://user-images.githubusercontent.com/80155305/212384115-3875164a-3717-465e-a3cc-cda9b8913ea6.png" width="250" height="230"/>
</p> 

<a name="end_effector_control"></a>
## End Effector control
Control of the gripper and pneumatic suction cup was implemented using ROS 2 services.

### Gripper:
Command example: 
```
ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: 'open', keep_compressor_running: true}"
```
Request fields:
- `gripper state` (type _string_) : _open/close_
- `keep_compressor_running` (type _bool_) : _true/false_

### Suction cup:
Command example:
```
ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"
```
Request fields:
- `enable_suction` (type _bool_) : _true/false_


<a name="additional"></a>
## Additional tools for visualization
The **dobot_visualization_tools** package provides visualization tools in form of RViz markers.  
1. Camera field of view (FOV) for Intel Realsense D435i (published at `/realsense_FOV` topic)
```
ros2 run dobot_visualization_tools show_camera_FOV
```
2. Workspace visualization (published at `/reachability_range` topic) - range without end effector attached.
```
ros2 run dobot_visualization_tools show_dobot_range
```
3. TCP trajectory (published at `/TCP_trajectory` topic) - trajectory markers are removed after 2 seconds of the manipulator's stationary state, so as not to slow RViz down with too many markers it needs to display. 
```
ros2 run dobot_visualization_tools show_trajectory
```
4. Interactive Markers - you can select the target position of the tool end (x, y, z, r) by moving the interactive marker in RViz. Once the target is selected, right-click on the yellow sphere and select the motion type from the menu. When running a node that allows control using interactive markers, two parameters must be specified (see example below) that define the point at the end of the tool in the coordinate system associated with the last link of the manipulator. 
```
ros2 run dobot_visualization_tools int_marker --ros-args -p TCP_x_offset:=0.059 -p  TCP_z_offset:=-0.12
```

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212383385-e9c2ca38-8cfa-4ac9-a106-99d2663ce629.png" width="220" height="220"/><img src="https://user-images.githubusercontent.com/80155305/212384560-198f5b65-b248-428d-baad-bf75c6909542.png" width="330" height="220"/>
</p> 

<p align="center">
  <img src="https://user-images.githubusercontent.com/80155305/212383482-6fa7b2c5-1bc1-4f0a-a118-a93dbc788a94.png" width="250" height="250"/><img src="https://user-images.githubusercontent.com/80155305/220213781-8e1b593d-4865-424c-9631-87724acba8d6.png" width="250" height="250"/>
</p> 

<video controls width="250">
    <source src="https://user-images.githubusercontent.com/80155305/219739487-8edd727b-aee9-4f14-b7c3-1d6c78ce2d4d.mp4">
</video>





<a name="examples"></a>
## Examples
Sample scripts have been included in **dobot_demos** package. Analysis of sample codes (_minimal working examples_) from this package will help you understand how to control Dobot Magician robot.  
Running sample scripts:  
- `ros2 run dobot_demos test_gripper`
- `ros2 run dobot_demos test_suction_cup`
- `ros2 run dobot_demos test_homing`
- `ros2 run dobot_demos test_point_to_point`
- `ros2 run dobot_demos test_pick_and_place`


<a name="rail"></a>
## Sliding rail 
If you have _Dobot Magician Sliding Rail_ and you want to get real time feedback about the position of the carriage you need to export `MAGICIAN_RAIL_IN_USE` environment variable before launching entire control stack (`export MAGICIAN_RAIL_IN_USE=true`). The current position of the carriage on the sliding rail will be published on the `/dobot_rail_pose` topic at a frequency of 20 Hz. After disconnecting the sliding rail, type `unset MAGICIAN_RAIL_IN_USE` and restart entire control system. 

Control of the sliding rail is possible both from **Dobot Magician Control Panel** RQT plugin and by using **/move_sliding_rail** action server. To launch **/move_sliding_rail** action server and load parameters (sliding rail velocity and acceleration) use the command below:
```
ros2 launch dobot_motion dobot_rail.launch.py
```
An example of a command that allows you to send a goal to an action server can be found below:
```
ros2 action send_goal /move_sliding_rail dobot_msgs/action/SlidingRail "{target_pose: 500}"
```

<a name="mrs"></a>
## Multi-Robot System (MRS) :robot:
If you want to connect several Dobot Magician robots to the same computer or to the same network, thus creating a _multi-robot system_ (MRS), check the [**magician-mrs**](https://github.com/jkaniuka/magician_ros2/tree/magician-mrs) branch.

<a name="video"></a>
## Video - see how the system works :movie_camera:

| Overview and features | Interactive Markers       | 
| ------ | ------ | 
| [<img src="https://user-images.githubusercontent.com/80155305/215295789-e6b1dadd-a819-4fe9-a633-4ce483a47964.png" width="100%">](https://vimeo.com/793400746)   | <video src="https://github.com/jkaniuka/dobot_tmp/assets/80155305/7b946f88-e4c8-499f-84f4-1ecf10534b25">| 




<a name="faq"></a>
## FAQ :question:
**Why haven't I used MoveIt 2?**  

MoveIt 2 is a great tool, but I have not used it in my control system for several reasons: 
* To begin with, this system was created with the idea of using it when learning ROS 2. First, you need to familiarize yourself with entities such as actions, topics, services, parameters to then understand how this is being utilized in MoveIt 2.
* Secondly, MoveIt is used among other things to generate collision-free trajectories. The mechanical design of Dobot Magician robot and its very short range (32 cm) makes it _de facto_ unsuitable for working in spaces with obstacles.
* Lastly, MoveIt enables grasp generation. Dobot Magician is equipped with a gripper that can be either fully open or fully closed. In addition, it is always pointed horizontally downward. Grip generation in this case does not seem to be necessary. 

<a name="citing"></a>
## Citing :scroll:
If you find this work useful, please give credits to the author by citing:

```
@mastersthesis{jkaniuka-bsc-23-twiki,
  author = {Kaniuka, Jan},
  school = {WEiTI},
  title = {{System sterowania robota manipulacyjnego Dobot Magician na bazie frameworka ROS2}},
  engtitle = {{Control system of DobotMagician robotic manipulator based on ROS2 framework}},
  year = {2023},
  type = {Bachelor's thesis},
  lang = {pl},
  organization = {IAiIS},
  publisher = {IAiIS},
  tutor = {Tomasz Winiarski},
  twiki = {bsc},
  url = {https://gitlab-stud.elka.pw.edu.pl/robotyka/rpmpg_pubs/-/raw/main/student-theses/jkaniuka-bsc-23-twiki.pdf}
}
```

<a name="contributing"></a>
## Contributing

#### Bug Reports & Feature Requests

Please use the [**issue tracker**](https://github.com/jkaniuka/magician_ros2/issues) to report any bugs or feature requests.




