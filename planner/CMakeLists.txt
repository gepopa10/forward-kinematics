project(planner)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(PLANNER_SRC
    src/distance_travel_time_finder.cpp
    src/trajectory_planner.cpp
)

add_library(planner_lib ${PLANNER_SRC})

target_include_directories(planner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/mock)
target_link_libraries(planner_lib kinematics_lib)

add_subdirectory(unittests)
add_subdirectory(functionaltests)