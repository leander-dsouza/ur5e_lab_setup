# ur5e_lab_setup
A collection of ROS2 packages required to run the ur5e arm.

## Installation

* Clone this repository and the moveit_task_constructor repo into your workspace:
	```bash
	cd ~/ros2_ws/src
	git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2
	```

* Install dependencies:
	```bash
	rosdep install --from-paths src --ignore-src -r -y.
	```

* Build the workspace:
	```bash
	colcon build --symlink-install
	```

## Usage

* Launch the ur5e arm in gazebo with MoveIt2:
	```bash
	ros2 launch mit_robot_moveit_config moveit.launch.py
	```

* Add the table and cube to the planning scene:
	```bash
	ros2 run mit_robot_control pub_planning_scene
	```

* Finally, run the MTC node for pick and place:
	```bash
	ros2 launch mit_robot_control pickplace.launch.py
	```
