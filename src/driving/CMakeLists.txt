cmake_minimum_required(VERSION 3.0.2)
project(driving)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include
#  LIBRARIES driving
CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}

)
add_executable(${PROJECT_NAME} src/driving_node.cpp
        src/FrameGrabber.cpp
        src/LaneDetector.cpp
        include/driving/FrameGrabber.h
        include/driving/LaneDetector.h
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
