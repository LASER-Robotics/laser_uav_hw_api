cmake_minimum_required(VERSION 3.8)
project(laser_uav_hw_api)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

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
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(px4_msgs QUIET)
find_package(ardupilot_msgs QUIET)

set(common_dependencies
    tf2
    tf2_msgs
    tf2_ros
    Eigen3
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    laser_msgs
    )

include_directories(include ${EIGEN3_INCLUDE_DIR})

set(exported_dependencies ${common_dependencies})

# ==============================================================================
# PX4 API BUILD
# ==============================================================================
if (px4_msgs_FOUND)
  message(STATUS "px4_msgs found! Building PX4 API support...")

  set(px4_dependencies ${common_dependencies} px4_msgs)
  list(APPEND exported_dependencies px4_msgs)

  add_executable(px4_api src/px4_api_main.cpp)
  add_library(px4_api_core SHARED src/px4_api_node.cpp)
  target_link_libraries(px4_api px4_api_core)
  ament_target_dependencies(px4_api_core ${px4_dependencies})

  target_include_directories(px4_api_core PRIVATE src/include)
  rclcpp_components_register_nodes(px4_api_core "laser_uav_hw_api::Px4ApiNode")

  install(
    TARGETS px4_api_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )
  install(TARGETS px4_api RUNTIME DESTINATION lib/${PROJECT_NAME})
else ()
  message(WARNING "px4_msgs NOT found! Skipping PX4 API build.")
endif ()

# ==============================================================================
# ARDUPILOT API BUILD
# ==============================================================================
if (ardupilot_msgs_FOUND)
  message(STATUS "ardupilot_msgs found! Building ArduPilot API support...")

  set(ap_dependencies ${common_dependencies} ardupilot_msgs)
  list(APPEND exported_dependencies ardupilot_msgs)

  add_executable(ap_api src/ap_api_main.cpp)
  add_library(ap_api_core SHARED src/ap_api_node.cpp)
  target_link_libraries(ap_api ap_api_core)
  ament_target_dependencies(ap_api_core ${ap_dependencies})

  target_include_directories(ap_api_core PRIVATE src/include)
  rclcpp_components_register_nodes(ap_api_core "laser_uav_hw_api::ApApiNode")

  install(
    TARGETS ap_api_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )
  install(TARGETS ap_api RUNTIME DESTINATION lib/${PROJECT_NAME})
else ()
  message(WARNING "ardupilot_msgs NOT found! Skipping ArduPilot API build.")
endif ()

# ==============================================================================
# COMMON INSTALLS & EXPORTS
# ==============================================================================
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include)
ament_export_dependencies(${exported_dependencies})
ament_package()
