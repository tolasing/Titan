cmake_minimum_required(VERSION 3.8)
project(imu_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(imu_node src/imu_node.cpp src/comms.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs serial)

target_link_libraries(imu_node serial)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME})

  install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
