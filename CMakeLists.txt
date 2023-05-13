cmake_minimum_required(VERSION 3.10)
project(uORB)

file(GLOB UORB_TOPIC_SRCS topics_sources/*.cc)

add_library(uORB SHARED
        uorbNew/device_master.cc
        uorbNew/device_node.cc
        uorbNew/uorb.cc
        ${UORB_TOPIC_SRCS}
        uORB.cpp
        )

find_package(Eigen3 REQUIRED)

target_include_directories(uORB PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR})

target_compile_options(uORB PRIVATE
        -std=c++11)

target_link_libraries(uORB)