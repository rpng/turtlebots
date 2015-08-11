cmake_minimum_required(VERSION 2.8.3)
project(slam_apriltag_isam_single)



find_package(catkin REQUIRED COMPONENTS
  tf
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED) 


add_message_files(
  DIRECTORY msg
  FILES PoseStampedArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES april_publish
#  CATKIN_DEPENDS OpenCV cv_bridge genmsg image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
INCLUDE_DIRS include
  CATKIN_DEPENDS  image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge std_msgs pcl_ros nav_msgs message_runtime
  DEPENDS Eigen3
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
# /usr/local/include
# ~/apriltags/build/include
)
link_directories(
#   /usr/local/lib # Location when using 'make system_install'
# ~/apriltags/build/lib
)

add_subdirectory(thirdparty)
link_libraries(isamlib)
link_libraries(apriltags)


add_executable(pose_publisher src/pose_publisher.cpp)
target_link_libraries(pose_publisher ${catkin_LIBRARIES})
add_executable(isam_receiver src/isam_receiver.cpp)
target_link_libraries(isam_receiver ${catkin_LIBRARIES}) 


