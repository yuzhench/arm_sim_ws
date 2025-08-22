cmake_minimum_required(VERSION 3.8)
project(arm_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)
find_package(pluginlib REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/simple_joint_control.py
  DESTINATION lib/${PROJECT_NAME}
)


add_library(arm_hw SHARED
  src/arm_hw.cpp
  src/motor_controller.cpp
)
target_include_directories(arm_hw PUBLIC include)
ament_target_dependencies(arm_hw
  rclcpp
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface pluginlib.xml)

 
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

 


install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS arm_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES pluginlib.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
 
 
ament_package()
