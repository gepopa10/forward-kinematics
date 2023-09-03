find_package(GTest REQUIRED)

set(TEST_TYPE "functionaltests")

add_executable(${PROJECT_NAME}_${TEST_TYPE} 
trajectory_planner_${TEST_TYPE}.cpp
)

target_link_libraries(${PROJECT_NAME}_${TEST_TYPE} ${PROJECT_NAME}_lib ${GTEST_BOTH_LIBRARIES} pthread)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_${TEST_TYPE})