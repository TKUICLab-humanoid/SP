cmake_minimum_required(VERSION 2.8.3)
project(strategy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge 
	image_transport
	message_generation
	roscpp
	roslib
	rospy
	sensor_msgs
	std_msgs
	tku_msgs
	tku_libs
)

find_package(OpenCV)
find_package(OpenCV 3.1 REQUIRED)
set(TKU_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libtku_libs.so)
set(STRATEGYBASE_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libstrategybase.so)
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
#add_message_files(DIRECTORY msg
#	FILES
#)
add_message_files(DIRECTORY msg
FILES quickchange.msg
      aruco_matrix.msg
)
## Generate services in the 'srv' folder
#add_service_files(DIRECTORY srv 
#	FILES
#)
add_service_files(DIRECTORY srv FILES Loadquickchange.srv)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
# std_msgs
# roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
)


#install(TARGETS testing RUNTIME DESTINATION bin)
add_executable(strategy Kidsize_HuroCup/strategy_main.cpp
			Kidsize_HuroCup/TSprintinfo.cpp
)
add_dependencies(strategy ${catkin_EXPORTED_TARGETS})
target_link_libraries(strategy ${catkin_LIBRARIES} ${TKU_LIBRARIES} ${ROSCOMMUNICATION_LIBRARIES})

add_executable(arucoimage Kidsize_HuroCup/aruco.cpp
)
add_dependencies(arucoimage ${catkin_EXPORTED_TARGETS})
target_link_libraries(arucoimage ${catkin_LIBRARIES} ${TKU_LIBRARIES} ${ROSCOMMUNICATION_LIBRARIES} ${OpenCV_LIBRARIES})
