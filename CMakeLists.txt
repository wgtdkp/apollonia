cmake_minimum_required(VERSION 2.6)
project(apollonia)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors")
if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif (APPLE)

include_directories(src)
add_subdirectory(src)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
add_executable(apollonia main.cc)
target_link_libraries(
    apollonia
    apollonialib
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
)
