cmake_minimum_required(VERSION 3.0.2)
project(mmm_robot)

add_compile_options(-std=c++11)

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  message_generation
)

add_message_files(FILES estimateValue.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES mmm_robot CATKIN_DEPENDS message_runtime std_msgs DEPENDS Eigen3)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

#set(SOURCE src/mmm_robot_interface.cpp src/torque_calculater.cpp src/extended_kalman_filter.cpp)
#add_executable(mmm_robot_interface ${SOURCE})

add_executable(
  mmm_robot_interface 
  src/mmm_robot_interface.cpp 
  src/torque_calculater.cpp
  src/extended_kalman_filter.cpp
)

target_link_libraries(mmm_robot_interface ${catkin_LIBRARIES})

