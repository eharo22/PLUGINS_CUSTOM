cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(wind_tests_plugin LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find all required Gazebo dependencies
# This will create imported targets like gz-sim8::gz-sim8, gz-sensors8::gz-sensors8, etc.
find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(sdformat14 REQUIRED)

# Add your own plugin's include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add the plugin as a shared library
add_library(wind_tests_plugin SHARED src/WindTests.cc)

# Link against the Gazebo imported targets.
# This automatically handles include directories and libraries for all linked targets.
target_link_libraries(wind_tests_plugin PUBLIC
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-msgs10::gz-msgs10
  sdformat14::sdformat14
)

# Optional: Install the plugin
# GZ_SIM_PLUGINS_PATH is set by gz-sim8's config.cmake after find_package.
install(TARGETS wind_tests_plugin
  DESTINATION ${GZ_SIM_PLUGINS_PATH}
)