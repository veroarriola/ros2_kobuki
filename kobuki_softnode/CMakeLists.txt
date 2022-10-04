cmake_minimum_required(VERSION 3.8)
project(kobuki_softnode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

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



##
## fake_kobuki_component
##
# add_library(fake_kobuki_component SHARED
#   src/fake_kobuki.cpp
# )

# target_compile_definitions(fake_kobuki_component
#   PRIVATE "COMPOSITION_BUILDING_DLL"
# )

# ament_target_dependencies(fake_kobuki_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
# )

# rclcpp_components_register_nodes(fake_kobuki_component
#   "kobuki::FakeKobuki"
# )

# set(node_plugins "${node_plugins}kobuki::FakeKobuki;$<TARGET_FILE:fake_kobuki>\n")


##
## fake_kobuki_ros_component
##
# add_library(fake_kobuki_ros_component SHARED
#   src/fake_kobuki_ros.cpp
# )

# target_compile_definitions(fake_kobuki_ros_component
#   PRIVATE "COMPOSITION_BUILDING_DLL"
# )

# ament_target_dependencies(fake_kobuki_ros_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs"
# )

# rclcpp_components_register_nodes(fake_kobuki_ros_component
#   "kobuki::FakeKobuki"
# )

# set(node_plugins "${node_plugins}kobuki::FakeKobukiRos;$<TARGET_FILE:fake_kobuki_ros>\n")


##
## kobuki_softnode
##

add_executable(kobuki_softnode
  #src/mobile_base_composition.cpp
  src/fake_kobuki.cpp
  src/fake_kobuki_ros.cpp
)

# target_link_libraries(kobuki_softnode
#   fake_kobuki_component
#   fake_kobuki_ros_component
# )

ament_target_dependencies(kobuki_softnode
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "kobuki_ros_interfaces"
  "tf2"
  "tf2_geometry_msgs"
)


# install(TARGETS
#   fake_kobuki_component
#   fake_kobuki_ros_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
install(TARGETS
  kobuki_softnode
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include${PROJECT_NAME}/
)


ament_package()
