# Kristo_p_sandbox
This package is made for learning MoveIt 2 Move Group C++ Interface.

## Prerequisites
* ROS2 (Humble Hawksbill)

[ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html)

* MoveIt 2 source code and Colcon

[MoveIt2 and Colcon installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)


Make sure to have sourced ROS2 before you proceed

`source /opt/ros/humble/setup.bash`

or to automatically source it every time:

`echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc`


## Running with Panda
* Clone the package to your colcon workspace and build it.

* Make sure you have sourced the colcon workspace

`source ~/ws_moveit2/install/setup.bash` or `echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc`

* Visualize the panda arm in RViz

`ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=false`


* Use the following commands for demonstration of different goals:

Pose goal:

`ros2 run movegroup_interface_examples pose_goal`

Named goal:

`ros2 run movegroup_interface_examples named_goal`

Joint-space goal:

`ros2 run movegroup_interface_examples joint_space_goal`

Cartesian path:

`ros2 run movegroup_interface_examples cartesian_path`

Gripper open:

`ros2 run movegroup_interface_examples gripper_open`

Gripper joint value:

`ros2 run movegroup_interface_examples gripper_joint_value`
