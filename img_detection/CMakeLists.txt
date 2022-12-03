cmake_minimum_required(VERSION 3.0.2)
project(img_detection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( face_detection_train face_detection_train.cpp )
target_link_libraries( face_detection_train ${OpenCV_LIBS} )

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
)


