cmake_minimum_required(VERSION 3.0.2)
project(openai_ros)

find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  StringArray.msg
)

add_service_files(
  FILES
  Completion.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openai_ros
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
