cmake_minimum_required(VERSION 3.5)
project(realsense_cpp)

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
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(include)
include_directories(${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(pose_example
  src/pose_example.cpp
  src/local_map/local_map.cpp
)

target_link_libraries(pose_example
  ${realsense2_LIBRARY}
)
  
ament_target_dependencies(pose_example rclcpp Eigen3)

# Install binaries
install(TARGETS 
  pose_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament exports
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
