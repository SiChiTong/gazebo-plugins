cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project (LLSF_WORLD_PLUGIN)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "src/*.h")
file(GLOB_RECURSE PB_SOURCES "../libs/msgs/*.cpp")
file(GLOB_RECURSE PB_HEADERS "../libs/msgs/*.h")

set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
foreach (_headerFile ${PB_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()

list(REMOVE_DUPLICATES INCLUDE_DIRS)
include_directories(${INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(llsf_world SHARED ${SOURCES} ${PB_SOURCES})
target_link_libraries(llsf_world ${GAZEBO_libraries})
