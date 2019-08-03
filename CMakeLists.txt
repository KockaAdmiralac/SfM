cmake_minimum_required(VERSION 3.0)
Project(SfM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui ximgproc imgcodecs features2d xfeatures2d)
find_library(libplyxx NAMES "../libplyxx/liblibplyxx.a")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../libplyxx/libplyxx")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(HEADERS
    src/ply.hpp
    src/sift.hpp
    src/surf.hpp
    src/lk.hpp
    src/klt.hpp
    src/kanatani.hpp
    src/icp.hpp
)

set(SOURCES
    src/ply.cpp
    src/sift.cpp
    src/surf.cpp
    src/lk.cpp
    src/klt.cpp
    src/kanatani.cpp
    src/icp.cpp
    src/main.cpp
)

add_executable(SfM
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(SfM PRIVATE src)
target_link_libraries(SfM ${OpenCV_LIBS} libplyxx)
