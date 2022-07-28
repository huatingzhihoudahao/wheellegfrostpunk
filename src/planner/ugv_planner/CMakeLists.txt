cmake_minimum_required(VERSION 2.8.3)
project(ugv_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(OsqpEigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENT common io VISUALIZATION)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_generation
  message_filters
  sensor_msgs
  pcl_conversions
)

add_message_files(FILES
  Polynome.msg
	
)

generate_messages(DEPENDENCIES
	std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS OpenCV Eigen Boost mpc
 message_runtime
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ugv_planner_node
  src/ugv_planner_node.cpp 
  src/sdf_map.cpp 
  )

add_dependencies(ugv_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ugv_planner_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${OMPL_LIBRARIES}
${PCL_VISUALIZATION_LIBRARIES})

target_link_libraries(ugv_planner_node OsqpEigen::OsqpEigen)
target_link_libraries(ugv_planner_node /usr/lib/x86_64-linux-gnu/liblz4.so.1.7.1)





# add_library()
