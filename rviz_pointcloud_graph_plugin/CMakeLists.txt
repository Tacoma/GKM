## Start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(rviz_pointcloud_graph_plugin)
find_package(	catkin REQUIRED COMPONENTS 
		rviz
		sensor_msgs
		lsd_slam_msgs
		pcl_ros
)


catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})





## This plugin includes Qt widgets, so we must include Qt
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/point_cloud2_graph_display.h
)

## Specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/point_cloud2_graph_display.cpp
  ${MOC_FILES}
)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -g -march=native -O3 -ftree-vectorize -std=c++0x"
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` and specify the list of 
## source files we collected above in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})



## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

