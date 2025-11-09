find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_add_gmock(${PROJECT_NAME}_state_maintainer_test
    rr_state_maintainer_test.cpp
)

ament_add_gmock(${PROJECT_NAME}_rr_battery_state_service_test
    rr_battery_state_service_test.cpp
)

ament_add_gmock(${PROJECT_NAME}_rr_gps_service_test
    rr_gps_service_test.cpp
)

ament_add_gmock(${PROJECT_NAME}_rr_range_service_test
    rr_gps_service_test.cpp
)

target_link_libraries(${PROJECT_NAME}_state_maintainer_test  ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_rr_battery_state_service_test  ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_rr_gps_service_test  ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_rr_range_service_test  ${PROJECT_NAME}_lib)
