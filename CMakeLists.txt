cmake_minimum_required(VERSION 3.8)
project(rr_state_mgm_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rr_common_base REQUIRED)

  # src/rr_service_base.cpp
  # src/rr_battery_state_subscriber.cpp
  # src/rr_battery_state_service.cpp
  # src/rr_gps_subscriber.cpp
  # src/rr_gps_service.cpp
  # src/rr_image_service.cpp
  # src/rr_imu_service.cpp
  # 
  # src/rr_joystick_service.cpp
  # src/rr_navigation_service.cpp
  # src/rr_range_service.cpp
  # 
  # src/state_validator.cpp
add_library(${PROJECT_NAME}_lib STATIC
  src/state_manager.cpp
  src/rr_state_subscriber_base.cpp
  src/rr_joystick_subscriber.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp rclcpp rclcpp_lifecycle rr_interfaces "rr_common_base")

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp 
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)  

install(TARGETS 
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # TODO: this should be added back
  # add_subdirectory(test)
endif()

ament_package()
