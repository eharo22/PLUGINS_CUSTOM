cmake_minimum_required(VERSION 3.10)
project(my_plugin)

find_package(gz-sim8 REQUIRED)  # Adjust version as needed

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/MyPlugin.cc)
target_link_libraries(${PROJECT_NAME} gz-sim8::gz-sim8)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
