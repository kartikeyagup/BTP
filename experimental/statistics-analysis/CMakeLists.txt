cmake_minimum_required(VERSION 2.8.12)

project(sensitivity-analysis)

#########################################################
# FIND C++11 Support
#########################################################
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
#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
#########################################################
# FIND OPENCV
#########################################################
find_package(OpenCV REQUIRED)
#########################################################
# FIND GFLAGS
#########################################################
find_package(gflags REQUIRED)
#########################################################
# Include Files
#########################################################
include_directories(${OpenCV_INCLUDE_DIRS} include)

# add_library(common-cc src/common.cc)
add_library(helpers-cc src/simulator.cc
                         src/common.cc
                         src/statistics.cc
                         src/triangulate.cc)


add_executable(sensitivity scripts/sensitivity.cpp)
target_link_libraries(sensitivity  
                           helpers-cc)
target_link_libraries(sensitivity ${OPENGL_LIBRARIES} 
                           ${GLUT_LIBRARY} 
                           ${OpenCV_LIBS}
                           gflags)
