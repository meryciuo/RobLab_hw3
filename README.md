# HOMEWORK3
This repository contains the packages required for the homework 3.
Follow the instructions to run the packages properly.

## Build and source the packages
Clone the repository in your ROS2 workspace and build the packages.
build
```bash
colcon build
```
then source

```bash
source install/setup.bash
```

## EXERCISE 1
Open three terminals
1. Launch iiwa_bring up
```bash
ros2 launch iiwa_bringup iiwa.launch.py world_name:=sphere.world  use_vision:=true  initial_positions_file:=initial_sphere_positions.yaml
```
2.
```bash
ros2 run  ros2_opencv  ros2_opencv_node

```

3. rqt
```bash
rqt
```


## EXERCISE 2A POSITIONING
Open four terminals.
1. Launch iiwa_bringuo
```bash
ros2 launch iiwa_bringup iiwa.launch.py world_name:=aruco.world  use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller" initial_positions_file:=initial_positions.yaml
```
2. Detect the marker
```bash
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=world -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
3. Launch the node
```bash
ros2 run ros2_kdl_package kdl_vision_control --ros-args -p cmd_interface:=velocity -p cmd:=positioning
```
4. Observe marker detection
```bash
rqt
```

## EXERCISE 2A LOOK_AT_POINT
Open four terminals.
1. Launch iiwa_bringuo
```bash
ros2 launch iiwa_bringup iiwa.launch.py world_name:=aruco.world  use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller" initial_positions_file:=initial_positions.yaml
```
2. Detect the marker
```bash
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
3. Launch the node
```bash
ros2 run ros2_kdl_package kdl_vision_control --ros-args -p cmd_interface:=velocity -p cmd:=look_at_point
```
4. Observe marker detection
```bash
rqt
```

## EXERCISE 2B
Open four terminals.
1. Launch iiwa_bringuo
```bash
ros2 launch iiwa_bringup iiwa.launch.py world_name:=aruco.world  use_vision:=true command_interface:="effort" robot_controller:="effort_controller" initial_positions_file:=initial_positions.yaml
```
2. Detect the marker
```bash
ros2 run aruco_ros single --ros-args -r /image:=/videocamera -r /camera_info:=/camera_info -p marker_id:=201 -p marker_size:=0.1 -p reference_frame:=camera_link_optical -p marker_frame:=aruco_marker_frame -p camera_frame:=camera_link_optical
```
3. Launch the node
```bash
ros2 run ros2_kdl_package ros2_kdl_node
```
4. Observe marker detection
```bash
rqt
```


