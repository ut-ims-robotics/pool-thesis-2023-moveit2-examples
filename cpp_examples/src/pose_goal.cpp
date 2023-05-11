#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("pose_goal");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pose_goal");

  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Create a target Pose for the end-effector
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
