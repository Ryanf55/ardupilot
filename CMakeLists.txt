# cmake -B build -DCONFIG_HAL_BOARD=3
# cmake --build build
cmake_minimum_required(VERSION 3.23)
project(ardupilot)

add_subdirectory(modules/mavlink)
add_subdirectory(libraries)


