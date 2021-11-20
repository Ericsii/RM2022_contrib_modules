cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(not ${CMAKE_BUILD_TYPE})
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DDEBUG_MODE")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# build shared lib
add_library(${PROJECT_NAME} SHARED
  "src/armor_detector_svm.cpp"
  "src/auto_aim_node.cpp"
)

set(dependencies
OpenCV
rm_interfaces
rm_cam
rclcpp
rclcpp_components
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# build executable
add_executable(auto_aim_node "src/auto_aim_node_main.cpp")
target_link_libraries(auto_aim_node ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "rm_auto_aim::AutoAimNode")

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS auto_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch resource config directory
install(
    DIRECTORY launch resource config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
