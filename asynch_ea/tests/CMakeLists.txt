cmake_minimum_required(VERSION 3.0)

add_executable(test test.cpp)
target_link_libraries(test PUBLIC ea_rg)
target_include_directories(test PUBLIC ../include/)

add_executable(view_random_robot view_random_robot.cpp)
target_link_libraries(view_random_robot PUBLIC ea_rg)
target_include_directories(view_random_robot PUBLIC ../include/)

add_executable(test_robot_distance test_robot_distance.cpp)
target_link_libraries(test_robot_distance PUBLIC ea_rg robotdesign)
target_include_directories(test_robot_distance PUBLIC ../include/)
