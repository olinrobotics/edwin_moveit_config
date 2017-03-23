cmake_minimum_required(VERSION 2.8.3)
project(edwin_moveit_config)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/edwin_moveit_config/cmake/modules/")

MESSAGE( STATUS "CMAKE_SOURCE_DIR:         " ${CMAKE_SOURCE_DIR} )

find_package(LibSerial REQUIRED)
find_package(Threads)
find_package(catkin REQUIRED
	roscpp
	control_msgs
	trajectory_msgs
	actionlib
	urdf
	joint_limits_interface
	transmission_interface
	hardware_interface
	controller_manager	
	moveit_ros_move_group
	moveit_ros_planning
	moveit_ros_planning_interface
	)

catkin_package()

add_executable(edwin_hardware_node src/edwin_interface.cpp src/st_arm.cpp src/utils.cpp src/Mutex.cpp)

add_executable(edwin_move_group_interface src/move_group_interface.cpp)

include_directories(
	include/
	${catkin_INCLUDE_DIRS}
	${LIBSERIAL_INCLUDE_DIR}
	#${Gflags_INCLUDE_DIRS}
	)

target_link_libraries(edwin_hardware_node
	${catkin_LIBRARIES}
	${LIBSERIAL_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
	)

target_link_libraries(edwin_move_group_interface
	${catkin_LIBRARIES}
	)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
