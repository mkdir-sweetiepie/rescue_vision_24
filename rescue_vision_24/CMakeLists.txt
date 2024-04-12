cmake_minimum_required(VERSION 3.0.2)
project(rescue_vision_24)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
cv_bridge
sensor_msgs
image_transport
)
find_package(OpenCV 4.5.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rescue_vision_24
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qr src/qr.cpp)
add_executable(hazmat src/hazmat.cpp)
add_executable(findc_motion src/findc_motion.cpp)
add_executable(master src/master.cpp)

target_link_libraries(qr
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)
target_link_libraries(hazmat
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${OpenCV_DNN_LIBS}
)
target_link_libraries(findc_motion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)
target_link_libraries(master
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${OpenCV_DNN_LIBS}
)
