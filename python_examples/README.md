This package is made for learning MoveIt 2 MoveItPy library.

Keep in mind that Python support is still in development for MoveIt 2.

## Running with Panda
* Make sure you have this package in your workspace and it is built with Colcon.

* Make sure you have sourced ROS and your workspace every time you open a terminal window.

* Visualize the Panda Arm in RViz

`ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=false`

>You can use the interactive markers to set goal poses for the robot and then press "Plan & execute" to move the robot.

* Demonstration of different goals

In another terminal, use the following commands for demonstration of different goals. Feel free to read the commented code for each example to understand how it works.

>Note that we are not just running a script, but instead launching a launch file, which also initializes configuration files that are required for using MoveIt2 with Python.
>
>Additionally, using MoveIt 2 with Python lacks some functionalities that the C++ interface has. Because of this, there are less examples for Python.

**Pose goal** - setting a target pose for the end-effector:

`ros2 launch python_examples python_examples.launch.py example_file:=pose_goal`

**Named goal** - setting a target pose that is previously defined in panda.srdf configuration file:

`ros2 launch python_examples python_examples.launch.py example_file:=named_goal`

**Joint-space goal** - setting a value to each joint of the robot:

`ros2 launch python_examples python_examples.launch.py example_file:=joint_goal`
