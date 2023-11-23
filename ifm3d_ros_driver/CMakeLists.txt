cmake_minimum_required(VERSION 3.5)
project(ifm3d_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ifm3d 1.2.6 REQUIRED COMPONENTS
             device
             framegrabber
             )

find_package(catkin REQUIRED COMPONENTS
             cv_bridge
             ifm3d_ros_msgs
             image_transport
             message_runtime
             nodelet
             roscpp
             rospy
             rostest
             sensor_msgs
             std_msgs
             tf2_geometry_msgs
             tf2_ros
             )

# catkin_python_setup()

option(CATKIN_ENABLE_TESTING "Build tests" OFF)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ifm3d_ros registration
  CATKIN_DEPENDS roscpp nodelet
  )

#############
##  Build  ##
#############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(ifm3d_ros src/camera_nodelet.cpp)
target_link_libraries(ifm3d_ros
  ${catkin_LIBRARIES}
  ifm3d::device
  ifm3d::framegrabber
  )

add_library(registration src/registration_nodelet.cpp)
target_link_libraries(registration
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS
  ifm3d_ros
  registration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/example_registration_2d_3d.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

######################
## Node-level tests ##
######################

if (CATKIN_ENABLE_TESTING)
  add_rostest(test/ifm3d.test)
  catkin_add_nosetests(test)
endif()
