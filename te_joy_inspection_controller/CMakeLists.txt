cmake_minimum_required(VERSION 2.8.3)
project(te_joy_inspection_controller)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  tf
  eigen_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  CATKIN_DEPENDS joy roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  controller_node.h
  deadzone.h
)

## Declare a C++ library
# add_library(controller
#   src/${PROJECT_NAME}/controller.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(controller_node src/controller_node.cpp)

## Add cmake target dependencies of the executable
# add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(controller_node
   ${catkin_LIBRARIES}
)
