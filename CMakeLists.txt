cmake_minimum_required(VERSION 3.10)

enable_testing()

project(forward_kinematics)

add_subdirectory(kinematics)

add_executable(main_app main.cpp)

target_link_libraries(main_app kinematics_lib)