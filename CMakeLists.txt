cmake_minimum_required(VERSION 2.8.12)
project(FIRE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARA 11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -fopenmp -rdynamic") # is or nor support openmp

include_directories(inc)
aux_source_directory(. SRC_CUR)
aux_source_directory(src SRC_SRC)
set(SOURCES ${SRC_CUR} ${SRC_SRC})

find_package(OpenCV 4.8.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 

add_executable(fire ${SOURCES})
target_link_libraries(fire ${OpenCV_LIBS})

set(RKNN_API_PATH ${CMAKE_SOURCES_DIRS}/home/orangepi/Downloads/rknpu-master/rknpu-master/rknn/rknn_api/librknn_api)
include_directories(/home/orangepi/Downloads/rknpu-master/rknpu-master/rknn/rknn_api/librknn_api/include)
set(RKNN_API_LIB ${RKNN_API_PATH}/${LIB_ARCH}/home/orangepi/Downloads/rknpu-master/rknpu-master/rknn/rknn_api/librknn_api/librknn_api.so)