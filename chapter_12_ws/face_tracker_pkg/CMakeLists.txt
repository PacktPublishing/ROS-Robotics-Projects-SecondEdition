cmake_minimum_required(VERSION 2.8.3)
project(face_tracker_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  face_tracker_control
  message_generation

)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui objdetect)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #/home/ros-projects/face_ws1/devel/include
)


add_executable(face_tracker_node src/face_tracker_node.cpp)
target_link_libraries(face_tracker_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

