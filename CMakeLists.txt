cmake_minimum_required(VERSION 2.8.3)
project(ls_global_location)

add_compile_options(-std=c++11)

find_package(catkin
    REQUIRED COMPONENTS
            roscpp
            std_msgs
            geometry_msgs
            sensor_msgs
            nav_msgs
            tf
)

catkin_package(
    INCLUDE_DIRS
            include
    CATKIN_DEPENDS
            roscpp
            std_msgs
            geometry_msgs
            sensor_msgs
            nav_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ls_global_location src/ls_global_location.cpp)
add_dependencies(ls_global_location ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ls_global_location ${catkin_LIBRARIES})
