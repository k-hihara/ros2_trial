cmake_minimum_required(VERSION 3.5)
project(ros2_trial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# PCLの依存関係
#find_package(PCL REQUIRED COMPONENTS common search)
find_package(PCL 1.7 REQUIRED COMPONENTS common segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add include directories
include_directories(include)

# Create an executable
add_executable(astar_planner src/astar.cpp)
add_executable(costmap src/costmap.cpp)
add_executable(dbscan src/dbscan.cpp)
add_executable(random_pcl_publisher src/random_pcl_publisher.cpp)
add_executable(save_costmap src/save_costmap.cpp)

# Add dependencies to the executable
ament_target_dependencies(astar_planner
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
)
ament_target_dependencies(costmap
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
)
ament_target_dependencies(dbscan
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "pcl_conversions"
)
ament_target_dependencies(random_pcl_publisher
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "pcl_conversions"
)
ament_target_dependencies(save_costmap
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
)
target_link_libraries(dbscan ${PCL_LIBRARIES})
target_link_libraries(random_pcl_publisher ${PCL_LIBRARIES})

# Install the executable
install(TARGETS astar_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS costmap
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS save_costmap
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS dbscan
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS random_pcl_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()