# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_gripper_control)
find_package(catkin REQUIRED COMPONENTS 
  controller_manager
  diagnostic_updater
  hardware_interface
  message_generation 
  robotiq_ethercat 
  roscpp 
  rospy
)

find_package(PkgConfig)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################
add_message_files(
  FILES
  Robotiq2FGripper_robot_input.msg
  Robotiq2FGripper_robot_output.msg
)

catkin_python_setup()

generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
      controller_manager
      diagnostic_updater
      message_runtime
      hardware_interface
      robotiq_ethercat 
      roscpp 
      rospy 
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/robotiq_2f_gripper_api.cpp
  src/${PROJECT_NAME}/robotiq_2f_gripper_diagnostics.cpp
  src/${PROJECT_NAME}/robotiq_2f_gripper_hw_interface.cpp
  src/${PROJECT_NAME}/robotiq_2f_gripper_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotiq_2f_gripper_ethercat_node
  src/${PROJECT_NAME}/robotiq_2f_gripper_ethercat_client.cpp
  src/${PROJECT_NAME}/robotiq_2f_gripper_ethercat_node.cpp
)

target_link_libraries(robotiq_2f_gripper_ethercat_node
  ${robotiq_ethercat_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(robotiq_2f_gripper_tcp_node
  src/${PROJECT_NAME}/robotiq_2f_gripper_tcp_client.cpp
  src/robotiq_2f_gripper_tcp_node.cpp
)

target_link_libraries(robotiq_2f_gripper_tcp_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_dependencies(robotiq_2f_gripper_ethercat_node 
  ${PROJECT_NAME}_generate_messages_cpp)
  
add_dependencies(robotiq_2f_gripper_tcp_node 
  ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS robotiq_2f_gripper_ethercat_node ${PROJECT_NAME} robotiq_2f_gripper_tcp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS nodes/Robotiq2FGripperRtuNode.py nodes/Robotiq2FGripperSimpleController.py nodes/Robotiq2FGripperStatusListener.py nodes/Robotiq2FGripperTcpNode.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/${PROJECT_NAME}/robotiq_2f_gripper_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)