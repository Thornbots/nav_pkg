cmake_minimum_required(VERSION 3.8)
project(thornbots_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dji_serial_bridge REQUIRED)

# Pure logic (no rclcpp), unit-tested standalone.
add_library(route_core STATIC src/route_core.cpp src/square_core.cpp)
target_compile_features(route_core PUBLIC cxx_std_17)
target_include_directories(route_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(waypoint_nav src/waypoint_nav_node.cpp)
target_link_libraries(waypoint_nav route_core)
ament_target_dependencies(waypoint_nav rclcpp geometry_msgs nav_msgs std_msgs dji_serial_bridge)

install(TARGETS waypoint_nav DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route_core test/test_route_core.cpp)
  target_link_libraries(test_route_core route_core)
  ament_add_gtest(test_square_core test/test_square_core.cpp)
  target_link_libraries(test_square_core route_core)

  # Launch-file lint; see setup.cfg for why the anyio plugin is disabled.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(thornbots_nav_pytest test TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

ament_package()
