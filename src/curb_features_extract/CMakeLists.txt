cmake_minimum_required(VERSION 3.0.2)
project(curb_features_extract)

set(BUILD_TYPE Release) # Release or Debug

add_compile_options(-march=native)
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    add_compile_options(-mfma)
endif ()

set(CMAKE_CXX_STANDARD 14)
if (${BUILD_TYPE} STREQUAL "Release")
    set(CMAKE_BUILD_TYPE Release)
    add_definitions(-DBUILD_TYPE_RELEASE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
    set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fPIC -pthread -fopenmp -Wno-error=deprecated-declarations -Wno-deprecated-declarations")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -pthread -fopenmp")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

elseif (${BUILD_TYPE} STREQUAL "RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
else ()
    set(CMAKE_BUILD_TYPE Debug)
    add_definitions(-DBUILD_TYPE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
endif ()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
INCLUDE_DIRS include
#  LIBRARIES curb_features_extract
CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
)

add_subdirectory(src/)
