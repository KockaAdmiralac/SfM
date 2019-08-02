cmake_minimum_required(VERSION 3.15.1)
Project(SfM)

if (WIN32)
    set(OpenCV_DIR "c:/opencv/build")
else()
    set(OpenCV_DIR "/usr/include/opencv4")
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui ximgproc imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS})

set(HEADERS
    src/klt.hpp
)

set(SOURCES
    src/klt.cpp
    src/main.cpp
)

add_executable(SfM
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(SfM PRIVATE src)
target_link_libraries(SfM ${OpenCV_LIBS})
