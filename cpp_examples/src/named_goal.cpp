#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "named_goal",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("named_goal");

  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target by name, these are defined in panda.srdf
  // Example named targets are "ready","extended","transport"
  move_group_interface.setNamedTarget("extended");

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
