cmake_minimum_required(VERSION 3.0)

project(RobotDesign)


# Use double precision in Bullet
add_compile_options(-DBT_USE_DOUBLE_PRECISION)
find_package(Torch REQUIRED)
add_subdirectory(core)
add_subdirectory(asynch_ea)
add_subdirectory(python_bindings)
if(WITH_RENDERING)
add_subdirectory(rule_viewer)
endif()
file(COPY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})
