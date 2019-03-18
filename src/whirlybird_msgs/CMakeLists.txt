cmake_minimum_required(VERSION 2.8.3)
project(whirlybird_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Whirlybird.msg
  Command.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


