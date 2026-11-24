cmake_minimum_required(VERSION 3.10)
project(autonomous_landing)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(X11 REQUIRED)

# PID controller: pure math, no ROS dependency
add_library(velocity_pid src/velocity_pid.cpp)
target_include_directories(velocity_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(velocity_pid PUBLIC cxx_std_17)
target_link_libraries(velocity_pid PUBLIC Eigen3::Eigen)

# Offboard node
add_executable(velocity_setpoint_control
  src/main.cpp
  src/velocity_setpoint_control.cpp)
target_compile_features(velocity_setpoint_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(velocity_setpoint_control velocity_pid)
ament_target_dependencies(
  velocity_setpoint_control
  "rclcpp"
  "px4_msgs"
  "geometry_msgs"
  )

# Re-stamps CameraInfo to match each Image (fixes "not synchronized" in apriltag_ros)
add_executable(camera_info_sync src/camera_info_sync.cpp)
target_compile_features(camera_info_sync PUBLIC cxx_std_17)
ament_target_dependencies(camera_info_sync "rclcpp" "sensor_msgs")

# Keyboard teleop (reads global key state through X11)
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_compile_features(keyboard_teleop PUBLIC cxx_std_17)
target_link_libraries(keyboard_teleop X11::X11)
ament_target_dependencies(keyboard_teleop "rclcpp" "px4_msgs")

install(TARGETS velocity_pid velocity_setpoint_control camera_info_sync keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

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
