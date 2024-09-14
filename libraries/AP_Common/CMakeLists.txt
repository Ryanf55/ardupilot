get_filename_component(_library_path ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
file(RELATIVE_PATH _lib_name ${_library_path} ${CMAKE_CURRENT_SOURCE_DIR})
add_library(${_lib_name})
add_library(${PROJECT_NAME}::${_lib_name} ALIAS ${_lib_name})

# ls -1 *.h
list(APPEND _headers
    AP_Common.h
    AP_ExpandingArray.h
    AP_FWVersionDefine.h
    AP_FWVersion.h
    AP_Test.h
    Bitmask.h
    ExpandingString.h
    float16.h
    Location.h
    MultiHeap.h
    NMEA.h
    sorting.h
    time.h
    TSIndex.h
)
# ls -1 *.cpp
list(APPEND _sources
    AP_Common.cpp
    AP_ExpandingArray.cpp
    AP_FWVersion.cpp
    c++.cpp
    ExpandingString.cpp
    float16.cpp
    Location.cpp
    MultiHeap.cpp
    NMEA.cpp
    sorting.cpp
    time.cpp
)
target_sources(AP_Common PUBLIC
   ${_headers}
PRIVATE
    ${_sources}
)

target_link_libraries(${_lib_name} PUBLIC ${PROJECT_NAME}::AP_HAL ${PROJECT_NAME}::GCS_MAVLink)

target_compile_options(${_lib_name}
    PUBLIC
        "-fsingle-precision-constant"
)

