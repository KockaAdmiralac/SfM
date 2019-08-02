cmake_minimum_required(VERSION 3.0)
Project(SfM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB LIBPLYXX_SOURCES "../libplyxx/libplyxx/*.cpp")

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui ximgproc imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../libplyxx/libplyxx")

set(HEADERS
    src/klt.hpp
)

set(SOURCES
    src/klt.cpp
    src/main.cpp
)

add_library(libplyxx
    "../libplyxx/libplyxx/libplyxx.cpp"
)

add_executable(SfM
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(SfM PRIVATE src)
target_link_libraries(SfM ${OpenCV_LIBS} libplyxx)
