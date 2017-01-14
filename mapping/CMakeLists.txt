set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_C_COMPILER "/usr/bin/gcc")
cmake_minimum_required(VERSION 2.8.3)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
)

add_library(mapping src/Main.cpp src/Map.cpp src/Micromouse.cpp src/Sensor.cpp src/Utilities.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})

catkin_package(INCLUDE_DIRS inc
               LIBRARIES mapping
)

add_executable(main src/Main.cpp)
target_link_libraries(main mapping ${catkin_LIBRARIES})

install(DIRECTORY inc/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Check c++14
# REFERENCE: http://stackoverflow.com/questions/37364217/ros-packages-with-c14-lambda-support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()