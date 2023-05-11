# Kristo_p_sandbox
This repository is made for learning MoveIt 2.

It contains two packages with code examples that demonstrate the usage of MoveIt 2 through C++ and Python API for the Franka Emika Panda robot.

## Prerequisites
* **ROS2 (Humble Hawksbill)**

[ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html)

* **MoveIt 2 source code and Colcon**

[MoveIt2 and Colcon installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

>If you want to use MoveIt 2 with just C++, it is recommended to use the **humble** branch of MoveIt 2, as it is currently the latest stable release. In this case you can fully follow the installation guide.
>
>If you want to use MoveIt 2 with Python, you should use the **main** branch of MoveIt 2, as the **humble** branch does not yet contain support for Python API. Note that the **main** branch of MoveIt 2 is being actively developed and might not be stable.
>
>To use the **main** branch of MoveIt 2, when following MoveIt 2 installation guide, replace the line
>
>`git clone https://github.com/ros-planning/moveit2_tutorials -b humble --depth 1`
>
>with
>
>`git clone https://github.com/ros-planning/moveit2_tutorials -b main --depth 1`

* **Make sure you have sourced ROS.**

`source /opt/ros/humble/setup.bash`

or to source it automatically 

`echo '/opt/ros/humble/setup.bash' >> ~/.bashrc`

## Using C++ and Python code examples on Franka Emika Panda robot

>In the following commands, replace ws_moveit2 with the name of your workspace if it is different.

* **Clone this repository into your workspace.**

`cd ~/ws_moveit2/src`

`git clone https://github.com/ut-ims-robotics-sandbox/Kristo_p_sandbox.git`

* **Build the workspace. Note that building your workspace for the first time might take a long time, even up to a few hours.**

`cd ~/ws_moveit2`

`colcon build`

* **Make sure you have sourced your workspace.**

`source ~/ws_moveit2/install/setup.bash`

or to source it automatically 

`echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc`

* **Follow the guide in the README.md in either package to use the code examples.**

MoveIt 2 planning demonstrations are done on a Franka Emika Panda robot. If you wish to use xArm or UR robots, follow the guide below.

## Using xArm or UR robot

WIP

