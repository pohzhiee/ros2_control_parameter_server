cmake_minimum_required(VERSION 3.5)
project(ros2_control_parameter_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_parameter_server REQUIRED)
find_package(parameter_server_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ros2_control_parameter_server SHARED src/parameter_server.cpp)
target_include_directories(ros2_control_parameter_server PUBLIC include)
ament_target_dependencies(
  ros2_control_parameter_server
  rclcpp
  controller_parameter_server
  parameter_server_interfaces
)

install(
  TARGETS ros2_control_parameter_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(param_server_exec src/param_server_exec.cpp)
target_link_libraries(param_server_exec ros2_control_parameter_server)
ament_target_dependencies(param_server_exec rclcpp controller_parameter_server)
ament_export_include_directories(
  include
)
install(TARGETS
  param_server_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include)


ament_package()
