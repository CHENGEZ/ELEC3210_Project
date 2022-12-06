cmake_minimum_required(VERSION 3.0.2)
project(perception)


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
#  LIBRARIES perception
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

add_executable( perception_train src/perception_train.cpp )
target_link_libraries( perception_train ${OpenCV_LIBS} )
add_executable(perception_run src/perception_run.cpp)
target_link_libraries(perception_run ${catkin_LIBRARIES} ${OpenCV_LIBS}) 
