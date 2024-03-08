cmake_minimum_required(VERSION 3.5)
project(tf_modifier_cpp)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# Add executable
add_executable(tf_modifier_node src/tf_modifier_node.cpp)

# Include directories
target_include_directories(tf_modifier_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Linking
ament_target_dependencies(tf_modifier_node rclcpp tf2_msgs)

# Install
install(TARGETS
    tf_modifier_node
    DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
