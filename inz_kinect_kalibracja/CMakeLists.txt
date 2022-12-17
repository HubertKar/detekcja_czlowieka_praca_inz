cmake_minimum_required(VERSION 3.0.2)
project(inz_kinect_kalibracja)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(get_tcp_tf src/get_TCP_tf.cpp)
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries( get_tcp_tf ${catkin_LIBRARIES} )
