cmake_minimum_required(VERSION 3.8)
project(bpsubsea_oculus_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ping.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs
)

add_executable(oculus_driver_node
  src/oculus_driver_node.cpp
)
target_compile_features(oculus_driver_node PUBLIC cxx_std_17)
target_include_directories(oculus_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(oculus_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(oculus_driver_node "${cpp_typesupport_target}")

add_executable(oculus_fan_image_node
  src/oculus_fan_image_node.cpp
)
target_compile_features(oculus_fan_image_node PUBLIC cxx_std_17)
target_include_directories(oculus_fan_image_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(oculus_fan_image_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(oculus_fan_image_node "${cpp_typesupport_target}")

install(TARGETS oculus_driver_node oculus_fan_image_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
