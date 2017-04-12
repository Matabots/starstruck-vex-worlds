cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS roscpp
            geometry_msgs
            tf
            std_msgs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include ${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_BINARY_DIR})
                    
add_definitions(-std=c++11)

catkin_package()
add_library(Robot Robot.cpp) #create a library called Robot from Robot.cpp

add_executable(RunRobot RunRobot.cpp)#create executable called RunRobot from RunRobot.cpp

add_dependencies(RunRobot  ${catkin_EXPORTED_TARGETS})#make a dependency on the package since it is using messages and has build dependencies

target_link_libraries(RunRobot ${catkin_LIBRARIES} Robot)
