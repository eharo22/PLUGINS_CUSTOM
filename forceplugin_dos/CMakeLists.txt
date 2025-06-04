cmake_minimum_required(VERSION 3.10)
project(table_vertical_force)

find_package(gz-sim8 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/TableVerticalForce.cc)
target_link_libraries(${PROJECT_NAME} gz-sim8::gz-sim8)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)