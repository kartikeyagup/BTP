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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

#########################################################
# FIND OPENCV
#########################################################
find_package(OpenCV REQUIRED)
#########################################################
# FIND GFLAGS
#########################################################
find_package(gflags REQUIRED)
#########################################################
# FIND GLOG
#########################################################
find_package(glog REQUIRED)
#########################################################
# FIND CERES
#########################################################
find_package(Ceres REQUIRED)
#########################################################
# Include Files
#########################################################
include_directories(${OpenCV_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    include)

add_library(helpers-cc src/triangulation.cc
                       src/sequential_probability_ratio.cc
                       src/random.cc
                       src/pose_util.cc
                       src/fundamental_matrix_util.cc
                       src/five_point.cpp
                       src/estimate_uncalibrated_relative_pose.cc
                       src/estimate_twoview_info.cc
                       src/estimate_relative_pose.cpp
                       src/essential_matrix_utils.cc
                       src/eight_point_fundamental_matrix.cc
                       src/correspondance.cc)

add_executable(slam scripts/slam.cpp)

add_executable(main scripts/main.cpp)

add_executable(test1 src/place_side.cpp)

target_link_libraries(slam ${OpenCV_LIBS}
                           gflags
                           glog
                           ${CERES_LIBRARIES}
                           helpers-cc)

target_link_libraries(main ${OpenCV_LIBS}
                           gflags
                           glog
                           ${CERES_LIBRARIES}
                           helpers-cc)

target_link_libraries(test1 ${OpenCV_LIBS})
