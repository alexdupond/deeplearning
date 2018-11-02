cmake_minimum_required(VERSION 2.8.3)
project(feature_recon)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# setup.py is called during catkin build
catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  BodyPartElm.msg
  Person.msg
  Persons.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_generation message_runtime
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

install()


add_executable(feature_recon_node src/feature_recon.cpp src/feature_extractor/feature_extractor.cpp)
target_link_libraries(feature_recon_node ${catkin_LIBRARIES})
add_dependencies(feature_recon_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
