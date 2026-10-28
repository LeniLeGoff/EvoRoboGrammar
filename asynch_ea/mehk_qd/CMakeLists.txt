cmake_minimum_required(VERSION 3.0)

add_executable(mehk_qd mehk_qd.cpp)
target_link_libraries(mehk_qd PUBLIC ea_rg)
target_include_directories(mehk_qd PUBLIC ../include/)
