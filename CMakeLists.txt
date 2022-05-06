cmake_minimum_required(VERSION 3.17)
project(testCSSC)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(testCSSC main.cpp cssc/csscM.h cssc/csscM.cpp)

target_link_libraries(testCSSC ${PCL_LIBRARIES})