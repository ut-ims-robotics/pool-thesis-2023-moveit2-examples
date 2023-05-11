cmake_minimum_required(VERSION 3.8)
project(cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(pose_goal src/pose_goal.cpp)
target_include_directories(pose_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_goal PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(joint_goal src/joint_goal.cpp)
ament_target_dependencies(
  joint_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(cartesian_path src/cartesian_path.cpp)
ament_target_dependencies(
  cartesian_path
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(named_goal src/named_goal.cpp)
ament_target_dependencies(
  named_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(gripper_open src/gripper_open.cpp)
ament_target_dependencies(
  gripper_open
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(gripper_joint_value src/gripper_joint_value.cpp)
ament_target_dependencies(
  gripper_joint_value
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS pose_goal
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS joint_goal
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cartesian_path
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS named_goal
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS gripper_open
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS gripper_joint_value
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
