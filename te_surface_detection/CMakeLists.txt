cmake_minimum_required(VERSION 2.8.3)
project(te_surface_detection)

find_package(catkin REQUIRED COMPONENTS
  image_geometry
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  tf_conversions
  visualization_msgs
  lsd_slam_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/config.cfg
)

#find_package(Eigen REQUIRED)
find_package(sophus REQUIRED)

catkin_package(CATKIN_DEPENDS)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/surface_detection.cpp
			       # src/bspline_fitting.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


