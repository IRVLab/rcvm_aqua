cmake_minimum_required(VERSION 2.8.3)
project(rcvm_aqua)

find_package(catkin REQUIRED COMPONENTS
  rospy
  aquacore
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   Affirmative.srv
   Attention.srv
   Danger.srv
   DiveAscend.srv
   DiveDescend.srv
   DiveWait.srv
   FollowMe.srv
   ImLost.srv
   IndicateObject.srv
   Malfunction.srv
   Negative.srv
   Possible.srv
   RepeatLast.srv
   ReportEnergy.srv
   StayHere.srv
 )

generate_messages(
	DEPENDENCIES
	std_msgs
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
#  LIBRARIES rcvm
  CATKIN_DEPENDS 
    rospy 
    aquacore
    std_msgs
    message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
