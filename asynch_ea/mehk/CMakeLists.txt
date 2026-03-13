cmake_minimum_required(VERSION 3.0)

add_executable(mehk mehk.cpp)
target_link_libraries(mehk PUBLIC ea_rg)
target_include_directories(mehk PUBLIC ../include/)
