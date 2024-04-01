cmake_minimum_required(VERSION 3.25)

project(lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS -pthread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(TCPSocket REQUIRED TCPSocket)

include_directories(include)
add_executable(${PROJECT_NAME} src/main.cpp src/localization.h src/localization.cpp)

set(RPLIDAR_SDK_PATH "/home/isen/rplidar_sdk")

add_library(rplidar STATIC IMPORTED)

set_property(TARGET rplidar
  PROPERTY IMPORTED_LOCATION ${RPLIDAR_SDK_PATH}/output/Linux/Release/libsl_lidar_sdk.a)

target_include_directories(rplidar INTERFACE
  ${RPLIDAR_SDK_PATH}/sdk/include/
  ${RPLIDAR_SDK_PATH}/sdk/src/)

target_link_libraries(${PROJECT_NAME} TCPSocket)

target_link_libraries(${PROJECT_NAME} rplidar)
