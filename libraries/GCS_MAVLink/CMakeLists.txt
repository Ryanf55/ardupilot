
get_filename_component(_library_path ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
unset(_lib_name)
file(RELATIVE_PATH _lib_name ${_library_path} ${CMAKE_CURRENT_SOURCE_DIR})
add_library(${_lib_name})
add_library(${PROJECT_NAME}::${_lib_name} ALIAS ${_lib_name})

# ls -1 *.h
list(APPEND _headers
    ap_message.h
    GCS_config.h
    GCS_Dummy.h
    GCS.h
    GCS_MAVLink.h
    MAVLink_routing.h
    MissionItemProtocol_Fence.h
    MissionItemProtocol.h
    MissionItemProtocol_Rally.h
    MissionItemProtocol_Waypoints.h
)
# ls -1 *.cpp
list(APPEND _sources
    GCS_Common.cpp
    GCS.cpp
    GCS_DeviceOp.cpp
    GCS_Dummy.cpp
    GCS_Fence.cpp
    GCS_FTP.cpp
    GCS_MAVLink.cpp
    GCS_Param.cpp
    GCS_Rally.cpp
    GCS_serial_control.cpp
    GCS_ServoRelay.cpp
    GCS_Signing.cpp
    MAVLink_routing.cpp
    MissionItemProtocol.cpp
    MissionItemProtocol_Fence.cpp
    MissionItemProtocol_Rally.cpp
    MissionItemProtocol_Waypoints.cpp
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

option(CONFIG_HAL_BOARD "Which board type" 3)

set(_mavlink_bindir ${PROJECT_BINARY_DIR}/modules/mavlink)
set(_mavlink_incdir ${_mavlink_bindir}/ardu_include)
target_include_directories(${_lib_name}
    PUBLIC
        ${_mavlink_incdir}
        # $<BUILD_INTERFACE:>
)
include(CMakePrintHelpers)

cmake_print_properties(TARGETS ${_lib_name} PROPERTIES
                       INCLUDE_DIRECTORIES)

# target_link_libraries(${_lib_name} PUBLIC common.xml-v2.0) # This doesn't work

add_dependencies(${_lib_name} _mavlink_intermediate)

add_custom_target(_mavlink_intermediate DEPENDS common.xml-v2.0 all-xml-v1.0) # Unclear how to get all.xml

add_custom_command(
  TARGET _mavlink_intermediate
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${_mavlink_bindir}/include
    ${_mavlink_incdir}/include/mavlink # The weird include path in ArduPilot source code.
)

# have build/modules/mavlink/ include/v2.0/common/version.h
# need build/modules/mavlink/ include/mavlink/v2.0/common/version.h