cmake_minimum_required(VERSION 3.8)
project(laser_uav_px4_api)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)

set(dependencies
Eigen3
rclcpp 
rclcpp_lifecycle 
rclcpp_components
sensor_msgs
nav_msgs 
std_srvs 
px4_msgs
laser_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


set(executable_name api)

add_executable(${executable_name}
  src/api_main.cpp
)

set(library_name api_core)

add_library(${library_name} SHARED
  src/api_node.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  api_core
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PRIVATE src/include)

rclcpp_components_register_nodes(${library_name} "laser_uav_px4_api::ApiNode")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
