cmake_minimum_required(VERSION 3.5)
project(humanoid_motor_control_cpp)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(humanoid_motor_example
  src/example.cpp
  src/spi/spi_utils.cpp
  src/driver/spi_driver.cpp
  src/driver/virtual_driver.cpp
  src/modern_robotics/modern_robotics.cpp
  src/modern_robotics/kinematics.cpp
  src/modern_robotics/dynamics.cpp
  src/low_level_control/motor_control.cpp
  src/low_level_control/imu_state.cpp
  src/humanoid_control/kinematic/ankle_ik.cpp
  src/humanoid_control/arm_control.cpp
  src/humanoid_control/leg_control.cpp
)
ament_target_dependencies(humanoid_motor_example rclcpp Eigen3)

# Install binaries
install(TARGETS 
  humanoid_motor_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament exports
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
