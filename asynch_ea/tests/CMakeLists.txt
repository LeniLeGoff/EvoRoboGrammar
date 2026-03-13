cmake_minimum_required(VERSION 3.0)

add_executable(test test.cpp)
target_link_libraries(test PUBLIC ea_rg)
target_include_directories(test PUBLIC ../include/)

add_executable(view_random_robot view_random_robot.cpp)
target_link_libraries(view_random_robot PUBLIC ea_rg)
target_include_directories(view_random_robot PUBLIC ../include/)
