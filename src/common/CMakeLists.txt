# CMakeList.txt : CMake project for rplidar_driver, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

project(common LANGUAGES CXX VERSION 0.1)

# add_library(${PROJECT_NAME}_logger STATIC
# include/main.cpp)   
include_directories(
    /
)
add_executable(${PROJECT_NAME}_logger
include/main.cpp)   
target_link_libraries(${PROJECT_NAME}_logger 
serial)