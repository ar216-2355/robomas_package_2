cmake_minimum_required(VERSION 3.8)
project(robomas_package_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorFeedback.msg"
  "msg/MotorCmd.msg"
  "msg/MotorCmdArray.msg"
  "msg/CanFrame.msg"
  "msg/Esc.msg"
)

add_executable(receiver src/can_receiver_node.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

add_executable(sender src/set_robomas_value_node.cpp)
ament_target_dependencies(sender rclcpp std_msgs)

# Use rosidl_get_typesupport_target + target_link_libraries instead of rosidl_target_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(receiver "${cpp_typesupport_target}")
target_link_libraries(sender "${cpp_typesupport_target}")

install(TARGETS
  receiver
  sender
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES sender_params.yaml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()