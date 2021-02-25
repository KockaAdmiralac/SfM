cmake_minimum_required(VERSION 3.0)
Project(SfM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_library(libplyxx NAMES "../libplyxx/liblibplyxx.a")
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui ximgproc imgcodecs features2d xfeatures2d)
find_package(PCL 1.9 REQUIRED)

include_directories("../libplyxx/libplyxx")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(HEADERS
    # src/kdtree.hpp
    src/kitti.hpp
    src/ply.hpp
    src/ransac.hpp
    src/sift.hpp
    src/surf.hpp
    src/lk.hpp
    src/klt.hpp
    # src/kanatani.hpp
    # src/icp.hpp
)

set(SOURCES
    # src/kdtree.cpp
    src/kitti.cpp
    src/ply.cpp
    src/ransac.cpp
    src/sift.cpp
    src/surf.cpp
    src/lk.cpp
    src/klt.cpp
    # src/kanatani.cpp
    # src/icp.cpp
    src/main.cpp
)

set(SEQUENCE 4 CACHE STRING "KITTI sequence to process")
set(DISTANCE_THRESH 600 CACHE STRING "Threshold for filtering matches based on keypoints distance")
set(MIN_HESSIAN 400 CACHE STRING "Minimum Hessian constant")
set(RATIO_THRESH 0.7f CACHE STRING "Lowe's ratio threshold")
set(RANSAC_1 8 CACHE STRING "First parameter to RANSAC")
set(RANSAC_2 100 CACHE STRING "Second parameter to RANSAC")
set(RANSAC_3 23 CACHE STRING "Third parameter to RANSAC")
set(METHOD sift CACHE STRING "Feature detection method")
set(MATCHER flann CACHE STRING "Feature matching method")
set(RANSAC our CACHE STRING "Whether to use our or OpenCV's RANSAC for solvePnP")

add_definitions(${PCL_DEFINITIONS})
add_compile_definitions(SEQUENCE_NUMBER=${SEQUENCE})
add_compile_definitions(DISTANCE_THRESH=${DISTANCE_THRESH})
add_compile_definitions(MIN_HESSIAN=${MIN_HESSIAN})
add_compile_definitions(RATIO_THRESH=${RATIO_THRESH})
add_compile_definitions(RANSAC_1=${RANSAC_1})
add_compile_definitions(RANSAC_2=${RANSAC_2})
add_compile_definitions(RANSAC_3=${RANSAC_3})
if(METHOD STREQUAL "surf")
    message("Using SURF feature detector.")
    add_compile_definitions(DETECTOR=xfeatures2d::SURF)
elseif(METHOD STREQUAL "orb")
    message("Using ORB feature detector.")
    add_compile_definitions(DETECTOR=ORB)
elseif(METHOD STREQUAL "sift")
    message("Using SIFT feature detector.")
    add_compile_definitions(DETECTOR=SIFT)
else()
    message("No known method specified.")
endif()
if (RANSAC STREQUAL "opencv")
    message("Using OpenCV's solvePnP.")
    add_compile_definitions(USE_THEIR_SOLVEPNP=yes)
endif()
if(MATCHER STREQUAL "flann")
    message("Using FLANN-based feature matcher.")
    add_compile_definitions(MATCHING=FLANNBASED)
elseif(MATCHER STREQUAL "bruteforce")
    message("Using brute force feature matcher.")
    add_compile_definitions(MATCHING=BRUTEFORCE)
else()
    message("No known matcher specified.")
endif()

add_executable(SfM
    ${HEADERS}
    ${SOURCES}
)

add_executable(ICP
    src/ply.hpp
    src/ply.cpp
    src/icp.cpp
)

target_include_directories(SfM PRIVATE src)
target_include_directories(ICP PRIVATE src)
target_link_libraries(SfM libplyxx ${OpenCV_LIBS})
target_link_libraries(ICP libplyxx ${PCL_LIBRARIES})
