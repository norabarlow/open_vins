cmake_minimum_required(VERSION 2.8.8)

# Project name
project(floatx)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED)


# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            INCLUDE_DIRS src
            LIBRARIES floatx_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
)

##################################################
# Make the core library
##################################################
add_library(floatx_lib SHARED
        src/floatx.h
        src/floatx.cpp
        src/types.h
        src/costable_1.h
        src/costable_0_1.h
        src/costable_0_01.h
        src/costable_0_001.h
        src/costable_0_0001.h
        src/sintable_1.h
        src/sintable_0_1.h
        src/sintable_0_01.h
        src/sintable_0_001.h
        src/sintable_0_0001.h
)
target_include_directories(floatx_lib PUBLIC src)
