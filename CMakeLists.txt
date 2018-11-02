cmake_minimum_required(VERSION 2.8.3)
project(aruco_tf_lizi)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  aruco
  message_generation
  rosbag
  aruco_mapping
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)

include_directories(
  ${PROJECT_SOURCE_DIR}}/src/
)

SET(SOURCES_B ${PROJECT_SOURCE_DIR}/src/tf_broadcaster.cpp)
SET(SOURCES_S ${PROJECT_SOURCE_DIR}/src/save_bag.cpp)
SET(SOURCES_R ${PROJECT_SOURCE_DIR}/src/read_bag.cpp)
SET(SOURCES_V ${PROJECT_SOURCE_DIR}/src/view_markers.cpp)

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_mapping.h)

##generate_messages(DEPENDENCIES
##		              std_msgs
##                  geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(lizi_broadcaster ${SOURCES_B})
target_link_libraries(lizi_broadcaster ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(save_bag ${SOURCES_S})
target_link_libraries(save_bag ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(read_bag ${SOURCES_R})
target_link_libraries(read_bag ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(view_markers ${SOURCES_V})
target_link_libraries(view_markers ${ROS_LIBRARIES} ${catkin_LIBRARIES})
