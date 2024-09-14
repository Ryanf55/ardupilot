get_filename_component(_library_path ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
file(RELATIVE_PATH _lib_name ${_library_path} ${CMAKE_CURRENT_SOURCE_DIR})
add_library(${_lib_name})
add_library(${PROJECT_NAME}::${_lib_name} ALIAS ${_lib_name})

# ls -1 *.h
list(APPEND _headers
    AnalogIn.h
    AP_HAL_Boards.h
    AP_HAL.h
    AP_HAL_Macros.h
    AP_HAL_Main.h
    AP_HAL_Namespace.h
    CANIface.h
    Device.h
    DSP.h
    Flash.h
    GPIO.h
    HAL.h
    I2CDevice.h
    LogStructure.h
    OpticalFlow.h
    RCInput.h
    RCOutput.h
    Scheduler.h
    Semaphores.h
    SIMState.h
    SPIDevice.h
    Storage.h
    system.h
    UARTDriver.h
    Util.h
    WSPIDevice.h
)
# ls -1 *.cpp
list(APPEND _sources
    CANIface.cpp
    Device.cpp
    DSP.cpp
    GPIO.cpp
    HAL.cpp
    RCOutput.cpp
    Scheduler.cpp
    Semaphores.cpp
    SIMState.cpp
    Storage.cpp
    system.cpp
    UARTDriver.cpp
    Util.cpp
)
target_sources(${_lib_name}
    PUBLIC
    FILE_SET
        HEADERS
        BASE_DIRS ".."
        FILES
            ${_headers}
PRIVATE
    ${_sources}
)

# option(CONFIG_HAL_BOARD "Which board type" "CONFIG_HAL_LINUX")
target_compile_definitions(${_lib_name}
    PUBLIC
        # CONFIG_HAL_BOARD=3
        CONFIG_HAL_BOARD=HAL_BOARD_SITL
        CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_NONE
        __AP_LINE__=__LINE__
        HAL_LOGGING_ENABLED=0
        ENABLE_HEAP=1
)
