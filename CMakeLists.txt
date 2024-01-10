cmake_minimum_required(VERSION 3.10)
project(track_vo)

find_package(OpenCV REQUIRED)

set(OpenCV_LIBS
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_highgui
  opencv_features2d
  opencv_calib3d
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  image_transport
  rospy
  std_msgs
  tf
  mission_manager
  message_generation
)

# add_service_files(
#   FILES 
#   Alarms.srv
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES 
#   std_msgs
# )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include/track_vo
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)  

add_executable(track_piloting src/track_vo.cpp src/TrackVO.cpp)
target_link_libraries(track_piloting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(IMU_orientation src/IMU_orientation.cpp)
target_link_libraries(IMU_orientation ${catkin_LIBRARIES})

add_executable(ground_truth src/ground_truth.cpp)
target_link_libraries(ground_truth ${catkin_LIBRARIES})

# add_executable(navegation src/navegation.cpp)
# target_link_libraries(navegation ${catkin_LIBRARIES})