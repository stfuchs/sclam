cmake_minimum_required(VERSION 2.8.3)
project(sclam)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  ar_track_alvar_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
)

find_package(g2o REQUIRED)


## Uncomment this if the package has a setup.py.
# catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Observation.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sclam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sclam_node.cpp)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${g2o_core_LIBRARY}
  ${g2o_stuff_LIBRARY}
  ${g2o_ext_csparse_LIBRARY}
  ${g2o_csparse_extension_LIBRARY}
  ${g2o_solver_csparse_LIBRARY}
  ${g2o_types_slam3d_LIBRARY}
)

add_executable(alvar_marker_observer_node src/alvar_marker_observer_node.cpp)

add_dependencies(alvar_marker_observer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(alvar_marker_observer_node
  ${catkin_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
