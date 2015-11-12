cmake_minimum_required(VERSION 2.8.3)
project(baxter_vision_example)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    message_generation
    roseus
    rosserial_arduino
    rosserial_client
    rostest
    std_msgs
)

include_directories(
      ${catkin_INCLUDE_DIRS}
)

catkin_package()
