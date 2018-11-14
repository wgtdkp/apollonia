cmake_minimum_required(VERSION 2.6)
project(apollonia)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if (NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")
endif ()

# OpenGL is deprecated in Mac OS mojave
if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif (APPLE)

include_directories(src)
add_subdirectory(src)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(glfw)

find_package(OpenGL REQUIRED)

add_executable(apollonia main.cc)
target_link_libraries(
    apollonia
    apollonialib
    glfw
    ${OPENGL_gl_LIBRARY}
)
