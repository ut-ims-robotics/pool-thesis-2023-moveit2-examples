# MoveIt 2 examples

<img src="https://moveit.ros.org/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>

This repository is made for learning MoveIt 2.

It contains two packages with code examples that demonstrate the usage of MoveIt 2 through C++ and Python API for the Franka Emika Panda robot.

> When using the commands provided in this guide, replace ws_moveit2 with the name of your workspace if it is different.

## Prerequisites
* **ROS2 (Humble Hawksbill)**

[ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html)

* **MoveIt 2 source code and Colcon**

[MoveIt2 and Colcon installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

It is important that we use MoveIt 2 Tutorials as our source, because it also contains the description for the Panda robot and visualising tools.

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

<img src="http://www.bolee.com.hk/wp-content/uploads/2021/09/Franka_Panda_204-scaled.jpg" alt="Franka Emika Panda" width="480"/>

* **Make sure you have installed ROS2 and created a workspace with Colcon that includes MoveIt 2 tutorials.**

This was covered in the Prerequisites chapter.

* **Clone this repository into your workspace.**

`cd ~/ws_moveit2/src`

`git clone https://github.com/ut-ims-robotics-sandbox/Kristo_p_sandbox.git`

* **Build the workspace. Note that building your workspace for the first time might take a long time, even up to a few hours.**

`cd ~/ws_moveit2`

`colcon build`

* **Make sure you have sourced your workspace. You have to source your workspace each time you open a new terminal window.**

`source ~/ws_moveit2/install/setup.bash`

or to source it automatically 

`echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc`

* **Follow the guide in the README.md in either package to use the code examples.**

MoveIt 2 planning demonstrations are done on a Franka Emika Panda robot. If you wish to use other robots, follow the guide below.

## MoveIt 2 with other robots

By default, [MoveIt 2 Tutorials](https://github.com/ros-planning/moveit2_tutorials) source that we use comes with support for Franka Emika Panda, Fanuc M-10iA and PR2 robots.

To use xArm robot, follow the guide below. For other robots, [follow this guide](https://moveit.picknik.ai/humble/doc/examples/examples.html#integration-with-a-new-robot).

## Using MoveIt 2 with xArm robots

<img src="https://cdn.shopify.com/s/files/1/0573/1483/6648/products/uarm-xarm-6-robotic-arm-1_600x.jpg?v=1681914771" alt="xArm 6" height="320"/>

>You can view the in-depth guide [here](https://github.com/xArm-Developer/xarm_ros2/tree/humble).

* **To use MoveIt 2 with xArm robots, clone the xArm source to the source directory of your workspace.**

`cd ~/ws_moveit2/src`

`git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO`

* **Update "xarm_ros2" repository.**

`cd ~/ws_moveit2/src/xarm_ros2`

`git pull`

`git submodule sync`

`git submodule update --init --remote`

* **Install dependencies.**

`cd ~/ws_moveit2/src/`

`rosdep update`

`rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y`

* **Build your workspace.**

`cd ~/ws_moveit2/`

`colcon build`

* **Launch xArm planner node.**

> The following commands are for xArm6. To use xArm 5 or 7, change "xarm6" to "xarm5" or "xarm7" respectively.

To visualize xArm robot in RViz:

`ros2 launch xarm_planner xarm6_planner_fake.launch.py add_gripper:=true`

or when using real xArm6

`ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.xxx.xxx [add_gripper:=true]`

* **Enter commands for demonstration of different goals.**

> The following commands are for xArm 6. To use xArm 5 or 7 change "dof:=6" to "dof:=5" or "dof:=7" respectively.

**Pose goal** - setting a target pose for the end-effector:

`ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6 robot_type:=xarm`

**Joint-space goal** - setting a value to each joint of the robot:

`ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=6 robot_type:=xarm`
