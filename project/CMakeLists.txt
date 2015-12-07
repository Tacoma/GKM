cmake_minimum_required(VERSION 2.8.3)
project(project)

find_package(catkin REQUIRED COMPONENTS
  image_geometry
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  message_generation
)

#find_package(Eigen REQUIRED)
find_package(sophus REQUIRED)
find_package(message_generation)

add_message_files(
	FILES 
	keyframeGraphMsg.msg
	keyframeMsg.msg
) 

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/PcMeshBuilder.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
