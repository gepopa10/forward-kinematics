project(kinematics)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(KINEMATICS_SRC
    src/forward_kinematics_solver.cpp
    src/kinematic_chain_factory.cpp
    src/collision_checker.cpp
)

add_library(kinematics_lib ${KINEMATICS_SRC})

target_include_directories(kinematics_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(functionaltests)
add_subdirectory(unittests)