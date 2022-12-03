cmake_minimum_required(VERSION 3.0.2)
project(img_detection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
)

find_package( OpenCV REQUIRED )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_detection
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( face_detection_train src/face_detection_train.cpp )
target_link_libraries( face_detection_train ${OpenCV_LIBS} )
add_executable(face_detection_run src/face_detection_run.cpp)
target_link_libraries(face_detection_run ${catkin_LIBRARIES} ${OpenCV_LIBS}) 
