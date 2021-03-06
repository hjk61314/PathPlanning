SET(execName PathPlanning)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(${execName})


SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

ADD_EXECUTABLE(${execName} ${HDR_FILES} ${SRC_FILES})
add_definitions(-std=c++11 -g -O3)

TARGET_LINK_LIBRARIES(${execName} ${OpenCV_LIBRARIES})
