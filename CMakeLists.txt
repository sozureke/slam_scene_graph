cmake_minimum_required(VERSION 3.5)
project(sg_slam)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g -O0)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_core
  src/joy_to_cmd_vel.cpp
  src/semantic_graph.cpp
  src/slam_handler.cpp
  src/cloud_handler.cpp
  src/marker_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}_core ${Boost_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_core rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs Boost visualization_msgs)


add_executable(semantic_graph_node_exec src/semantic_graph_node.cpp)
target_link_libraries(semantic_graph_node_exec ${PROJECT_NAME}_core)
ament_target_dependencies(semantic_graph_node_exec rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs Boost visualization_msgs)


install(TARGETS
  semantic_graph_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
