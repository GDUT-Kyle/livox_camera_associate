cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_associate)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  livox_ros_driver
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED QUIET)
find_package(Eigen3 3.3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES faster_lio_sam
 CATKIN_DEPENDS cv_bridge geometry_msgs livox_ros_driver nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

add_executable(colorLidarDisplay src/colorLidarDisplay.cpp)
add_dependencies(colorLidarDisplay ${catkin_EXPORTED_TARGETS})
target_link_libraries(colorLidarDisplay ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
