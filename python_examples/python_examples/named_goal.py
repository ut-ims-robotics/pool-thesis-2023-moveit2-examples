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

    # Set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # Set target goal using predefined state, these are defined in panda.srdf
    # Example named targets are "ready","extended","transport"
    panda_arm.set_goal_state(configuration_name="extended")

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