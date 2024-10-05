cmake_minimum_required(VERSION 3.5)
project(rrt_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)


# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})  # Add Eigen3 include directory



set(lifecycle_bond_dependencies
    rclcpp
    bond
    bondcpp
    rclcpp_lifecycle)


################################################################

add_library(lifecycle_bond_utils src/lifecycle_bond_utils/lifecycle_bond_utils.cpp )
# target_link_libraries(lifecycle_bond_utils
#       rmf_traffic::rmf_traffic
#       nlohmann_json_schema_validator
#   )
ament_target_dependencies(
  lifecycle_bond_utils
    ${lifecycle_bond_dependencies}
  )
target_include_directories(lifecycle_bond_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(lifecycle_bond_utils PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++

ament_export_targets(lifecycle_bond_utils HAS_LIBRARY_TARGET)
# install(
#   DIRECTORY include
#   DESTINATION include
# )
 install(TARGETS lifecycle_bond_utils
 EXPORT lifecycle_bond_utils
 LIBRARY DESTINATION lib
 ARCHIVE DESTINATION lib
 RUNTIME DESTINATION bin
 INCLUDES DESTINATION include/lifecycle_bond_utils
 )

##################################################################




################################################################
add_executable(rrt_planner_node src/rrt_planner/rrt_planner_node.cpp)

target_include_directories(rrt_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rrt_planner_node
  lifecycle_bond_utils
)
ament_target_dependencies(rrt_planner_node rclcpp bond bondcpp lifecycle_msgs std_msgs geometry_msgs nav_msgs rclcpp_lifecycle visualization_msgs Eigen3) 

#################################################################

add_executable(rrt_star_planner_node src/rrt_planner/rrt_star_planner_node.cpp)

target_include_directories(rrt_star_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rrt_star_planner_node
  lifecycle_bond_utils
)
ament_target_dependencies(rrt_star_planner_node rclcpp bond bondcpp lifecycle_msgs std_msgs geometry_msgs nav_msgs rclcpp_lifecycle visualization_msgs Eigen3) 

###############################################################################


#################################################################

add_executable(rrt_connect_planner_node src/rrt_planner/rrt_connect_planner_node.cpp)

target_include_directories(rrt_connect_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rrt_connect_planner_node
  lifecycle_bond_utils
)
ament_target_dependencies(rrt_connect_planner_node rclcpp bond bondcpp lifecycle_msgs std_msgs geometry_msgs nav_msgs rclcpp_lifecycle visualization_msgs Eigen3) 

###############################################################################


add_executable(map_publisher src/rrt_planner/map_publisher.cpp)
ament_target_dependencies(map_publisher rclcpp nav_msgs)


##############################################################################

add_executable(pose_publisher src/rrt_planner/pose_publisher.cpp)
ament_target_dependencies(pose_publisher rclcpp geometry_msgs)
###############################################################################
# Install targets
install(TARGETS
  rrt_star_planner_node
  rrt_planner_node
  rrt_connect_planner_node
  map_publisher
  pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# install the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
