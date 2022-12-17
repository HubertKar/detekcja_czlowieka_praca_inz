cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(inz_detekcja_czlowieka)

find_package(PCL 1.10 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  octomap_msgs
  tf
  geometry_msgs
)

catkin_package()


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

find_package(octomap REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

#Searching CUDA
find_package(CUDA)

#Include the FindCUDA script
include(FindCUDA)

message ("=========== FCL LIB ${LIBFCL_LIBRARIES}")
link_libraries(${OCTOMAP_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(links_colison_detection src/links_colision_detection.cpp)
add_executable(workspace_colision_detection src/workspace_colision_detection.cpp)
cuda_add_executable (people_detection src/people_detection.cpp)

target_link_libraries (people_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries (links_colison_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries (workspace_colision_detection ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES})