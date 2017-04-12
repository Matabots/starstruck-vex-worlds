cmake_minimum_required(VERSION 2.8.3)
project(jetson)

find_package(catkin REQUIRED COMPONENTS roscpp
            geometry_msgs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-std=c++11)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_BINARY_DIR})
                    
add_executable(Euler Euler.cpp)

add_dependencies(Euler ${catkin_EXPORTED_TARGETS})

target_link_libraries(Euler ${catkin_LIBRARIES})
