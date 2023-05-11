#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("cartesian_path");

  // We spin up a SingleThreadedExecutor to get current pose of the robot later
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  //x - forward(+) and backward(-)
  //y - left(+) and right(-)
  //z - up(+) and down(-)

  // Variable to hold waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Current pose
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;

  // Variable for first target pose waypoint
  geometry_msgs::msg::Pose target_pose = start_pose;

  // Move diagonally
  target_pose.position.x += 0.1; //Forward
  target_pose.position.z += 0.2; //Up

  // Add target pose to waypoints
  waypoints.push_back(target_pose); 

  // Variable for next target pose
  geometry_msgs::msg::Pose target_pose2 = target_pose;

  // Move only along one axis
  target_pose2.position.y -= 0.1; //Right

  // Add next target pose to waypoints
  waypoints.push_back(target_pose2);


  // We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian translation
  // We will specify the jump threshold as 0.0, effectively disabling it
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
  // Computing the Cartesian path, which is stored in trajectory
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

  // Execute the trajectory
  move_group_interface.execute(trajectory);

  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}