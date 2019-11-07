cmake_minimum_required(VERSION 2.8.3)
project(pose_to_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  # LIBRARIES blah
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs 
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
add_executable(pose_to_odom src/pose_to_odom.cpp)
target_link_libraries(pose_to_odom ${catkin_LIBRARIES})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY param
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY maps
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY rviz
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
