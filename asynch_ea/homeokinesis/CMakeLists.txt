cmake_minimum_required(VERSION 3.0)

add_executable(homeokinesis homeokinesis.cpp)
target_link_libraries(homeokinesis PUBLIC ea_rg)
target_include_directories(homeokinesis PUBLIC ../include/)
