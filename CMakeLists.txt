cmake_minimum_required(VERSION 3.8)
project(auto_mapper)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

set(deps
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        nav2_msgs
        tf2
        tf2_ros
        rclcpp_action
        nav2_costmap_2d
        nav2_map_server
        slam_toolbox
        visualization_msgs
        nav2_util
        std_msgs
        angles
        tf2_geometry_msgs
        map_msgs
        rclcpp_action
        nav2_msgs
        )
set(targets
        auto_mapper
        auto_mapper_cartographer
        send_goal_nav2
        marker_publisher
        )

foreach (d ${deps})
  find_package(${d} REQUIRED)
endforeach ()

foreach (t ${targets})
  add_executable(${t} src/${t}.cpp)
  ament_target_dependencies(${t} ${deps})
endforeach ()

# add_library(send_goal_action_client SHARED
#   src/send_goal_nav2.cpp)
# target_include_directories(send_goal_action_client PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(send_goal_action_client
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# ament_target_dependencies(send_goal_action_client
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "nav2_msgs")
# rclcpp_components_register_node(send_goal_action_client PLUGIN "autonomous_exploration_mapper::SendGoalNav2ActionClient" EXECUTABLE send_goal_nav2_action_client)
# install(TARGETS
#   send_goal_action_client
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)


install(TARGETS
        ${targets}
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY config launch rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
