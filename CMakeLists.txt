cmake_minimum_required(VERSION 3.8)
project(ardupilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------- #
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# --------------------------------------------------------------------------- #
# add custom targets to configure and build ardupilot

# TODO(srmainwaring): add cache variables for vehicle type, debug etc. 
# set(ARDUPILOT_VEHICLE "copter" CACHE STRING "Vehicle type")

find_program(WAF_COMMAND waf HINTS ${PROJECT_SOURCE_DIR})
message(STATUS "WAF_COMMAND: ${WAF_COMMAND}")

add_custom_target(ardupilot_configure ALL
  ${WAF_COMMAND} configure --board sitl --enable-dds --debug
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target(ardupilot_build ALL
  ${WAF_COMMAND} build --enable-dds --debug
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
add_dependencies(ardupilot_build ardupilot_configure)

# --------------------------------------------------------------------------- #
# build tests

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
# call last

ament_package()
