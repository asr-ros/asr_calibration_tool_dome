cmake_minimum_required(VERSION 2.8.3)
project(asr_calibration_tool_dome)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS 
	asr_msgs
	roscpp 
	sensor_msgs 
	std_msgs 
	image_transport
	checkerboard_detector
)

find_package(Boost REQUIRED COMPONENTS 
	system
	signals
)

catkin_package(
    CATKIN_DEPENDS asr_msgs roscpp sensor_msgs std_msgs image_transport checkerboard_detector
    DEPENDS Boost
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS})

add_executable(asr_calibration_tool_dome 
    src/DataTracker.cpp
)

# dependency to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this 
add_dependencies(asr_calibration_tool_dome asr_msgs_generate_messages_cpp)

    
target_link_libraries(
    asr_calibration_tool_dome 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)
