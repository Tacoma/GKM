cmake_minimum_required(VERSION 2.8.3)
project(surface_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
 # message_filters
  std_msgs
  geometry_msgs
  message_generation
)

#find_package(Eigen REQUIRED)
find_package(message_generation)

add_message_files(
	DIRECTORY msg
	FILES 
	Surface.msg
) 

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
