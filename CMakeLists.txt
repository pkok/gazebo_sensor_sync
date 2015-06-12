cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH /usr/local/lib/OGRE/cmake/)

include (FindPkgConfig)

find_package(gazebo REQUIRED)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OGRE_INCLUDE_DIR} ${OGRE_Paging_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_executable(listener listener.cc)
target_link_libraries(listener ${GAZEBO_LIBRARIES} gazebo_client)


add_library(pose_publisher SHARED pose_publisher.cc)
target_link_libraries(pose_publisher ${GAZEBO_LIBRARIES})
