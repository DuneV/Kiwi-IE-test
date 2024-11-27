cmake_minimum_required(VERSION 3.5)
project(interfaces)

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

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(ALSA REQUIRED)
find_package(usr_msgs REQUIRED)  # This line ensures usr_msgs is found

# Add ROS 2 message generation tools
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Declare dependencies
set(DEPENDENCIES
  rclcpp
  std_msgs
  utils
  ALSA
  usr_msgs  # Ensure usr_msgs is in the list of dependencies
)

# Generate ROS 2 interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/fail_detection/Fails.msg"  # Corrected path to your msg file
)

# Declare the executable
add_executable(interfaces_node
  src/interfaces_node.cpp
  src/modules/speaker.cpp
)

# Link libraries to the executable
target_link_libraries(interfaces_node
  utils::utils
)

# Include directories for the target
target_include_directories(interfaces_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(interfaces_node ${DEPENDENCIES})

# Install target and include directories
install(TARGETS interfaces_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/modules/
  DESTINATION include/modules/
)

# Export the dependencies to be used by other packages
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export include directories for the generated messages
ament_export_include_directories(include)
ament_package()
