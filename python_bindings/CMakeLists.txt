set(CMAKE_CXX_STANDARD 17)

set(Python3_FIND_VIRTUALENV "FIRST")
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

set(LIBRARY_NAME pyrobotdesign)
find_package(pybind11)
#set(PYBIND11_ROOT ${PROJECT_SOURCE_DIR}/extern/pybind11)
#add_subdirectory(${PYBIND11_ROOT} ${CMAKE_BINARY_DIR}/pybind11)
pybind11_add_module(${LIBRARY_NAME}
    py_eigen_geometry.cpp
    py_graph.cpp
    py_optim.cpp
    py_prop.cpp
    py_render.cpp
    py_robot.cpp
    py_robot_design.cpp
    py_sim.cpp
    py_value.cpp
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
    PREFIX ""
)

# Copy __init__.py and setup.py to the output directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py
    ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

target_link_libraries(${LIBRARY_NAME} PRIVATE
    robotdesign
)
install(TARGETS ${LIBRARY_NAME} 
        COMPONENT python
        LIBRARY DESTINATION "${${Python3_SITELIB}}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${LIBRARY_NAME}"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "${${Python3_SITELIB}}/bin")
