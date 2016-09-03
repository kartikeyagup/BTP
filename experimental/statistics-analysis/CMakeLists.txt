cmake_minimum_required(VERSION 2.8.12)

project(sensitivity-analysis)

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
                           ${OpenCV_LIBS})
