cmake_minimum_required(VERSION 3.8)
project(teleop_leaders)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  urdf
  kdl_parser
  orocos_kdl_vendor
  rosidl_default_generators
  teleop_msgs
  yaml-cpp
  pinocchio
)


find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY description launch include config
  DESTINATION share/${PROJECT_NAME}
)

generate_parameter_library(feedback_controller_parameters
  src/feedback_controller_parameters.yaml
)

add_library(feedback_controller SHARED
  src/feedback_controller.cpp
)
target_compile_features(feedback_controller PUBLIC cxx_std_17)
target_include_directories(feedback_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/feedback_controller>
)
target_link_libraries(feedback_controller PUBLIC
  feedback_controller_parameters
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${controller_interface_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
ament_target_dependencies(feedback_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(feedback_controller PRIVATE "feedback_controller_BUILDING_DLL" "_USE_MATH_DEFINES")
target_compile_definitions(feedback_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface feedback_controller.xml)

install(
  TARGETS feedback_controller
  DESTINATION lib/
)
install(
  DIRECTORY include/
  DESTINATION include/feedback_controller
)
install(TARGETS
    feedback_controller
    feedback_controller_parameters
  EXPORT export_feedback_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib

)

ament_export_include_directories(
  include
)

ament_export_targets(export_feedback_controller HAS_LIBRARY_TARGET)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
