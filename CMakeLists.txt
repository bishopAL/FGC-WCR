cmake_minimum_required(VERSION 2.8.9)
PROJECT(robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Torch_DIR ~/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
add_executable(torch_example ./src/main.cpp)
target_link_libraries(torch_example "${TORCH_LIBRARIES}")
set_property(TARGET torch_example PROPERTY CXX_STANDARD 17)