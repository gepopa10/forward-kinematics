find_package(GTest REQUIRED)

set(TEST_TYPE "unittests")
set(GMOCK_BOTH_LIBRARIES "/usr/local/lib/libgmock.a" "/usr/local/lib/libgmock_main.a")

add_executable(${PROJECT_NAME}_${TEST_TYPE} 
distance_travel_time_finder_${TEST_TYPE}.cpp
trajectory_planner_${TEST_TYPE}.cpp
)
target_link_libraries(${PROJECT_NAME}_${TEST_TYPE} ${PROJECT_NAME}_lib ${GTEST_BOTH_LIBRARIES} ${GMOCK_BOTH_LIBRARIES} pthread)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_${TEST_TYPE})