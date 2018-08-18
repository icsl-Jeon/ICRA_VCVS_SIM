cmake_minimum_required(VERSION 2.8.3)
project(asap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)

if(${gazebo_VERSION_MAJOR} GREATER 2)
    message(STATUS "Building iris.sdf.")
    
    set(enable_mavlink_interface "true")
    set(enable_ground_truth "false")
    set(enable_logging "false")
    set(enable_camera "false")
    set(enable_wind "false")
    set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
    set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
    
    # Creates shell commands to generate .sdf file
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
      COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
      DEPENDS ${rotors_description_dir}/urdf/iris.xacro
      DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
      DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
      )
    add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)
else()
  message(STATUS "Gazebo version is less than 3, cannot generate iris.sdf (gz tool not available).")
endif()

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building waypoint publisher or hovering example, returning now.")
  return()
endif()
#CVXGEN 
set(CVXGEN_DIR ~/Downloads/cvxgen)


#OCTOMAP 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})


#-------------------#
# ASAP dependencies #
#-------------------#


find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs octomap_ros mav_msgs roscpp sensor_msgs octomap_msgs message_generation kiro_gui_msgs trajectory_msgs)

add_service_files(
    FILES
	SolvePath.srv 
  )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   kiro_gui_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asap
#  CATKIN_DEPENDS other_catkin_pkg
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
${CVXGEN_DIR}
)


add_executable(asap_path_solver src/solve_path_test.cpp src/ASAP.cpp src/utils.cpp src/PolyTrajGen.cpp ${CVXGEN_DIR}/solver.c ${CVXGEN_DIR}/ldl.c ${CVXGEN_DIR}/matrix_support.c ${CVXGEN_DIR}/util.c)
target_link_libraries(asap_path_solver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} m)
add_dependencies(asap_path_solver ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/asap.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/asap_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


