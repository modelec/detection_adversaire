cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)

project(lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(CMAKE_CXX_FLAGS -pthread)

include_directories(include)
add_executable(${PROJECT_NAME} src/main.cpp src/localization.h src/localization.cpp)

set(RPLIDAR_SDK_PATH "/home/modelec/rplidar_sdk")

add_library(rplidar STATIC IMPORTED)

set_property(TARGET rplidar
  PROPERTY IMPORTED_LOCATION ${RPLIDAR_SDK_PATH}/output/Linux/Release/libsl_lidar_sdk.a)

target_include_directories(rplidar INTERFACE
  ${RPLIDAR_SDK_PATH}/sdk/include/
  ${RPLIDAR_SDK_PATH}/sdk/src/)

target_link_libraries(${PROJECT_NAME} rplidar)
