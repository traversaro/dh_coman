
cmake_minimum_required(VERSION 2.8)

PROJECT(dh_coman)

find_package(orocos_kdl)
find_package(kdl_format_io)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS})

add_executable(check_dh_coman dh_coman.cpp)

target_link_libraries(check_dh_coman ${orocos_kdl_LIBRARIES} ${kdl_format_io_LIBRARIES})
