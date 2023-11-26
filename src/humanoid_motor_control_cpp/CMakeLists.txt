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
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(humanoid_motor_example
  src/example.cpp
  src/spi/spi_driver.cpp
  src/spi/spi_utils.cpp
  src/kinematic/ankle_ik.cpp
  src/modern_robotics/modern_robotics.cpp
  src/modern_robotics/kinematics.cpp
  src/modern_robotics/dynamics.cpp
  src/motor_control/motor_control.cpp
  src/humanoid_control/arm_control.cpp
  src/humanoid_control/leg_control.cpp
)
ament_target_dependencies(humanoid_motor_example rclcpp Eigen3::Eigen)

install(TARGETS 
  humanoid_motor_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
