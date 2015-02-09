cmake_minimum_required(VERSION 2.8.3)
project(dolphin_slam)

# set default build type
# Debug|Release|RelWithDebInfo|MinSizeRel
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  time_monitor
  sensor_msgs
  std_msgs
  cv_bridge
  visualization_msgs
  message_generation
  image_transport
  rosbag
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#diretivas para usar o FabMAP oficial no codigo
add_subdirectory(third_part)
include_directories(third_part/FabMapCommon/include)
include_directories(third_part/pAcceleratedChowLiu_CompactMemory/include)
include_directories(third_part/FabMapV2/include)

find_package(VXL REQUIRED )
if (VXL_FOUND)
    message(STATUS "VXL found:  ${VXL_CMAKE_DIR}")
  include(${VXL_CMAKE_DIR}/UseVXL.cmake)
endif()

include_directories(${VXL_CORE_INCLUDE_DIR})


find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ImageHistogram.msg
   ActiveLocalViewCells.msg
   ExecutionTime.msg
   ExperienceEvent.msg
   PlaceCellActivity.msg
   Error.msg
   Descriptors.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ImageRequest.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dolphin_slam
  CATKIN_DEPENDS roscpp time_monitor message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/dolphin_slam
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  third_part

)

# Declare a cpp library
#add_library(dolphin_slam
#  src/${PROJECT_NAME}/dolphin_slam.cpp
#)

## Declare a cpp executable
add_executable(image_processing_node
    src/image_processing_node.cpp
    src/dolphin_slam/image_processing.cpp
)
target_link_libraries(image_processing_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(bow_training
    src/bow_training_node.cpp
    src/dolphin_slam/bow_descriptor_extractor.cpp
    src/dolphin_slam/bow_training.cpp
    src/dolphin_slam/utils.cpp
)
target_link_libraries(bow_training
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)


## Declare a cpp executable
add_executable(fabmap_training
    src/dolphin_slam/utils.cpp
    src/dolphin_slam/bow_descriptor_extractor.cpp
    src/fabmap_training_node.cpp
    src/dolphin_slam/fabmap_training.cpp
)

target_link_libraries(fabmap_training
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    vnl
    MOOS
    FabMapCommon
    ${STXXL_LIBRARIES}
    acceleratedChowLiu
)

### Declare a cpp executable
add_executable(local_view_node
    src/local_view_node.cpp
    src/dolphin_slam/local_view_module.cpp
    src/dolphin_slam/bow_descriptor_extractor.cpp
)
target_link_libraries(local_view_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    FabMapV2
)

#### Declare a cpp executable
add_executable(experience_map_node
    src/dolphin_slam/experience_map.cpp
    src/experience_map_node.cpp
    src/dolphin_slam/utils.cpp
)

target_link_libraries(experience_map_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

#### Declare a cpp executable
add_executable(place_cell_node
    src/dolphin_slam/place_cell_network.cpp
    src/place_cell_node.cpp
)
target_link_libraries(place_cell_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

### Declare a cpp executable
add_executable(robot_state_node
    src/dolphin_slam/robot_state.cpp
    src/robot_state_node.cpp
    src/dolphin_slam/utils.cpp
)

target_link_libraries(robot_state_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(bow_training dolphin_slam_generate_messages_cpp)
add_dependencies(fabmap_training dolphin_slam_generate_messages_cpp)
add_dependencies(local_view_node dolphin_slam_generate_messages_cpp)
add_dependencies(experience_map_node dolphin_slam_generate_messages_cpp)
add_dependencies(place_cell_node dolphin_slam_generate_messages_cpp)
add_dependencies(robot_state_node dolphin_slam_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(dolphin_slam_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dolphin_slam dolphin_slam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dolphin_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
