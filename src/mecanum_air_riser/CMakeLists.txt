cmake_minimum_required(VERSION 3.8)
project(mecanum_air_riser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drobo_interfaces REQUIRED)
find_package(usb_connect REQUIRED)
find_package(motor_lib REQUIRED)

include_directories(
  include
)

add_executable(mecanum_air_riser
  src/main.cpp
)

target_link_libraries(mecanum_air_riser
  usb_connect
  md_lib
  smd_lib
  sd_lib
  sr_lib
  sm_lib
)

ament_target_dependencies(mecanum_air_riser
  rclcpp
  drobo_interfaces
  motor_lib
)

install(TARGETS
  mecanum_air_riser
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
