cmake_minimum_required(VERSION 2.8.3)
project(arva_sim)

find_package(Gazebo REQUIRED)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  roslib
  roslib
  tf
	tf2_msgs
)

include_directories(
 ${GAZEBO_INCLUDE_DIRS}
 ${GAZEBO_PROTO_INCLUDE_DIRS}
 ${SDF_INCLUDE_DIRS}
)

add_message_files(
	FILES
  arva_data.msg
  arva.msg
)

generate_messages(
  DEPENDENCIES
	std_msgs
  geometry_msgs
)


catkin_package(
  DEPENDS 
  roscpp 
	gazebo_ros 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(arva_sim_transmitter src/transmitter_plugin.cpp)
target_link_libraries(arva_sim_transmitter ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library( arva_sim_receiver src/receiver_plugin.cpp )
target_link_libraries(arva_sim_receiver ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable( arva_trace_example example/example.cpp )
target_link_libraries(arva_trace_example ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


