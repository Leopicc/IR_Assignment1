cmake_minimum_required(VERSION 3.8)
project(group11_assignament_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------
# Find packages
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

# -----------------------------------------------------------------------
# Generate custom interfaces (srv + action)
# -----------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindApril.srv"
  "srv/FindTables.srv"
  DEPENDENCIES geometry_msgs
)

# Needed to use the generated interfaces in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -----------------------------------------------------------------------
# Executables
# -----------------------------------------------------------------------

# --- april_detection ---
add_executable(april_detection_node src/april_detection_node.cpp)
ament_target_dependencies(april_detection_node
  rclcpp
  geometry_msgs
  apriltag_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(april_detection_node ${cpp_typesupport_target})

# --- table_detection_node ---
add_executable(table_detection_node src/table_detection_node.cpp)
ament_target_dependencies(table_detection_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros               # ← aggiungi
  tf2_geometry_msgs     # ← aggiungi
)
target_link_libraries(table_detection_node ${cpp_typesupport_target})

# --- brain_node ---
add_executable(brain_node src/brain_node.cpp)
ament_target_dependencies(brain_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)
target_link_libraries(brain_node ${cpp_typesupport_target})

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------
install(TARGETS
  april_detection_node
  table_detection_node
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

