cmake_minimum_required(VERSION 3.0)


add_executable(view_robot view_robot.cpp)
target_link_libraries(view_robot PUBLIC ea_rg)
target_include_directories(view_robot PUBLIC ../include/)

add_executable(view_behavior view_behavior.cpp)
target_link_libraries(view_behavior PUBLIC ea_rg)
target_include_directories(view_behavior PUBLIC ../include/)
