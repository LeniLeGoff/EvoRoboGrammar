cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


file(GLOB_RECURSE SOURCES "src/*.cpp")
message(${SOURCES})

# find_package(apear)

add_compile_options(-DEIGEN_DONT_PARALLELIZE)
# set(EIGEN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/extern/eigen)
set(BULLET_ROOT ${PROJECT_SOURCE_DIR}/extern/bullet3)
set(BULLET_INCLUDE_DIRS ${BULLET_ROOT}/src ${BULLET_ROOT}/Extras)



add_library(ea_rg STATIC ${SOURCES})
target_link_libraries(ea_rg PUBLIC robotdesign apear tbb)
target_include_directories(ea_rg PUBLIC
    "include"
    "../core/include"
    "/usr/include/eigen3"
    ${BULLET_INCLUDE_DIRS}
)
target_compile_options(ea_rg PRIVATE -Wall -Werror)

add_subdirectory(tests)
