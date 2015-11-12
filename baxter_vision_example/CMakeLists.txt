cmake_minimum_required(VERSION 2.8.3)
project(baxter_vision_example)

find_package(catkin REQUIRED COMPONENTS
    rospy    
    euslisp
)

include_directories(
      ${catkin_INCLUDE_DIRS}
)

catkin_package()
