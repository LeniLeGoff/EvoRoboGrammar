set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


file(GLOB_RECURSE SOURCES "src/*.cpp")
message(${SOURCES})

# find_package(apear)

add_compile_options(-DEIGEN_DONT_PARALLELIZE)
# set(EIGEN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/extern/eigen)
set(BULLET_ROOT ${PROJECT_SOURCE_DIR}/extern/bullet3)
set(BULLET_INCLUDE_DIRS ${BULLET_ROOT}/src ${BULLET_ROOT}/Extras)

find_package(Eigen3 REQUIRED)

add_library(ea_rg STATIC ${SOURCES})
target_link_libraries(ea_rg PUBLIC Eigen3::Eigen robotdesign apear tbb)
target_include_directories(ea_rg PUBLIC
    "include"
    "../core/include"
    ${BULLET_INCLUDE_DIRS}
)
if(WITH_RENDERING)
    target_compile_definitions(ea_rg PUBLIC RENDERING)
endif()
target_compile_options(ea_rg PRIVATE -Wall -Werror)

add_subdirectory(tests)
add_subdirectory(mehk)
add_subdirectory(mehk_qd)
add_subdirectory(homeokinesis)
add_subdirectory(tools)
add_subdirectory(python_bindings)
