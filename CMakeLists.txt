cmake_minimum_required(VERSION 3.0.2)
project(openai_ros)

find_package(catkin REQUIRED COMPONENTS
   catkin_virtualenv REQUIRED
   rospy
   std_msgs
   message_generation
)

catkin_python_setup()

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

# openai>=0.11.0 Python module requries >= Python3.7.1 (latest openai module that works on Python3.6 was 0.10.5 (2021/10/2)
# check if python3.7 executable found
find_program(PYTHON3_PROGRAM NAMES python3.12 python3.11 python3.10 python3.9 python3.8 python3.7)
get_filename_component(PYTHON3_PROGRAM_NAME ${PYTHON3_PROGRAM} NAME)

if(PYTHON3_PROGRAM)
  message(STATUS "Python executable found: ${PYTHON3_PROGRAM}")
else()
  message(FATAL_ERROR "Python 3.7 executable not found, please install 'python3.7' 'python3.7-venv' via apt/rpm etc..")
endif()
# generate the virtualenv
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.txt
  PYTHON_INTERPRETER ${PYTHON3_PROGRAM_NAME}
  USE_SYSTEM_PACKAGES FALSE
  CHECK_VENV FALSE
  )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/openai_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
