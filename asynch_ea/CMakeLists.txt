cmake_minimum_required(VERSION 3.0)


file(GLOB_RECURSE SOURCES "*.cpp")
message(${SOURCES})

find_package(apear)

add_executable(ea_rg ${SOURCES})
target_link_libraries(ea_rg PUBLIC apear)
target_include_directories(ea_rg PUBLIC
    "include"
    "../core/include"
    "/usr/include/eigen3"
)
