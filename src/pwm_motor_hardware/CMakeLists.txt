cmake_minimum_required(VERSION 3.8)
project(pwm_motor_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(pwm_motor_hardware SHARED src/pwm_motor_hardware.cpp)

target_compile_features(pwm_motor_hardware PUBLIC cxx_std_17)
target_include_directories(pwm_motor_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/pwm_motor_hardware>
)
ament_target_dependencies(
  pwm_motor_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS pwm_motor_hardware
  EXPORT export_pwm_motor_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(hardware_interface pwm_motor_hardware.xml)
ament_export_targets(export_pwm_motor_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
