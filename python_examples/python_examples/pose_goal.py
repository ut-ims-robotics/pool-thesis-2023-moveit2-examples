#!/usr/bin/env python3

# Generic ROS libraries
import rclpy
from rclpy.logging import get_logger

# MoveIt python library
from moveit.planning import (
    MoveItPy,
)

def main():

    # Initialize rclpy and ROS logger
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # Instantiate MoveItPy and get planning component
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("panda_arm")

    # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # Create PoseStamped message that will hold the target Pose
    from geometry_msgs.msg import PoseStamped
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "panda_link0"

    # Describe the target pose for the end-effector
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5

    # Set the target pose
    panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")

    # Create a plan to the target pose
    plan_result = panda_arm.plan()

    # If the plan is successful, get the trajectory and execute the plan
    if plan_result:
        robot_trajectory = plan_result.trajectory
        panda.execute(robot_trajectory, blocking=True, controllers=[])
    else:
        logger.error("Planning failed")

    rclpy.shutdown()

if __name__ == "__main__":
    main()