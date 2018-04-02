project(tutorial-image)
cmake_minimum_required(VERSION 2.8)
find_package(VISP REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x")

include_directories(${VISP_INCLUDE_DIRS})
add_executable(example-executable lel.cpp)
target_link_libraries(example-executable ${VISP_LIBRARIES})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(example-executable ${PYTHON_LIBRARIES})
