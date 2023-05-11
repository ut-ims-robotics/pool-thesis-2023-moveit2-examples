This package is made for learning MoveIt 2 Move Group C++ Interface.

## Running with Panda
* Make sure you have this package in your workspace and it is built with Colcon.

* Make sure you have sourced ROS and your workspace every time you open a terminal window.

* Visualize the Panda Arm in RViz

`ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=false`

>You can use the interactive markers to set goal poses for the robot and then press "Plan & execute" to move the robot.

* Demonstration of different goals

In another terminal, use the following commands for demonstration of different goals. Feel free to read the commented code for each example to understand how it works.

**Pose goal** - setting a target pose for the end-effector:

`ros2 run cpp_examples pose_goal`

**Named goal** - setting a target pose that is previously defined in panda.srdf configuration file:

`ros2 run cpp_examples named_goal`

**Joint-space goal** - setting a value to each joint of the robot:

`ros2 run cpp_examples joint_goal`

**Cartesian path** - giving a set of waypoints that the robot follows:

`ros2 run cpp_examples cartesian_path`

**Gripper open** - opening the gripper of the robot:

`ros2 run cpp_examples gripper_open`

**Gripper joint value** - giving a custom value to set how much the gripper is opened:

`ros2 run cpp_examples gripper_joint_value`
