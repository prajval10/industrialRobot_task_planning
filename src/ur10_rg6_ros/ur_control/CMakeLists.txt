cmake_minimum_required(VERSION 2.8.3)
project(ur_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)


 add_service_files(
   FILES
   RG6.srv
   RG6_Grip.srv
 )



 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  DEPENDS system_lib
    roscpp
    std_msgs
    message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
