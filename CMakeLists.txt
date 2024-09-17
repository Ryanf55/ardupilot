# cmake -B build -DCONFIG_HAL_BOARD=3
# cmake --build build
cmake_minimum_required(VERSION 3.23)
project(ardupilot)


add_compile_definitions(${_lib_name}
    PUBLIC
        # CONFIG_HAL_BOARD=3
        CONFIG_HAL_BOARD=HAL_BOARD_SITL
        CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_NONE
        __AP_LINE__=__LINE__
        HAL_LOGGING_ENABLED=0
        ENABLE_HEAP=1
        APM_BUILD_TYPE=APM_BUILD_ArduPlane
)

add_subdirectory(modules/mavlink)
add_subdirectory(libraries)


