set(CMAKE_CXX_STANDARD 17)

set(LIBRARY_NAME py_async_ea)
find_package(pybind11)
#set(PYBIND11_ROOT ${PROJECT_SOURCE_DIR}/extern/pybind11)
#add_subdirectory(${PYBIND11_ROOT} ${CMAKE_BINARY_DIR}/pybind11)
pybind11_add_module(${LIBRARY_NAME}
    py_async_ea.cpp
    ../mehk_qd/mehk_qd.cpp
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
    PREFIX ""
)

#Copy __init__.py and setup.py to the output directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py
    ${CMAKE_CURRENT_BINARY_DIR}/setup.py
)

target_link_libraries(${LIBRARY_NAME} PRIVATE
    robotdesign ea_rg apear
)
target_include_directories(${LIBRARY_NAME} PUBLIC ../ ../include)
install(TARGETS ${LIBRARY_NAME} DESTINATION lib/.)
