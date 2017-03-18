cmake_minimum_required(VERSION 3.6)
project(LaneDetector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES LaneDetector.cpp LaneDetector.h)
add_executable(LaneDetector ${SOURCE_FILES})

#Add from here!
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package( OpenCV REQUIRED )
include_directories($(OpenCV_INCLUDE_DIRS))
target_link_libraries(LaneDetector ${OpenCV_LIBS})