cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(direction_custom_interface REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv" 
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
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

add_executable(patrol src/patrol.cpp)
ament_target_dependencies(patrol rclcpp sensor_msgs std_msgs)


add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp sensor_msgs std_msgs std_srvs )
# Get the typesupport target and link against it
# rosidl_get_typesupport_target(direction_service_typesupport ${PROJECT_NAME}_interfaces) # For all interfaces
# target_link_libraries(direction_service ${direction_service_typesupport})

add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp sensor_msgs std_msgs std_srvs )
# Get the typesupport target and link against it
# rosidl_get_typesupport_target(patrol_with_service_typesupport ${PROJECT_NAME}_interfaces) # For all interfaces
# target_link_libraries(patrol_with_service ${patrol_with_service_typesupport})

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp sensor_msgs std_msgs std_srvs )
# Get the typesupport target and link against it
# rosidl_get_typesupport_target(test_service_typesupport ${PROJECT_NAME}_interfaces) # For all interfaces
# target_link_libraries(test_service ${test_service_typesupport})

add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action rclcpp rclcpp_action tf2 nav_msgs geometry_msgs std_msgs )
# Get the typesupport target and link against it
# rosidl_get_typesupport_target(go_to_pose_action_typesupport ${PROJECT_NAME}_interfaces) # For all interfaces
# target_link_libraries(go_to_pose_action ${go_to_pose_action_typesupport})

# ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS
   patrol
   direction_service
   patrol_with_service
   test_service
   go_to_pose_action
   DESTINATION lib/${PROJECT_NAME}
 )
#  ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_target_interfaces(go_to_pose_action
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(direction_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(patrol_with_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(test_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")  

ament_package()
