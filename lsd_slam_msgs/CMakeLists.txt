cmake_minimum_required(VERSION 2.8.3)
project(lsd_slam_msgs)

find_package(catkin REQUIRED COMPONENTS
 # message_filters
  std_msgs
  message_generation
)

#find_package(Eigen REQUIRED)
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


#add_library(${PROJECT_NAME} ${SOURCE_FILES})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

