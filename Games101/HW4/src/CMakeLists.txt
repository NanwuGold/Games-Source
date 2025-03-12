cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)

add_executable(BezierCurve ${SOURCE_FILES})

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

set(CMAKE_BUILD_TYPE "Debug")