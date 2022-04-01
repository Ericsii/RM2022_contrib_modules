cmake_minimum_required(VERSION 3.8)
project(nahsor_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_definitions("-DRM_DEBUG_MODE")
endif()

set(g2o_DIR "/usr/local/lib/cmake/g2o/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_util REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rm_trajectory REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include
include_directories(include)

# Create nahsor_modules lib
add_library(${PROJECT_NAME} SHARED
  "src/nahsor_node.cpp"
  "src/nahsor_detector.cpp"
  "src/nahsor_prediction.cpp"
  "src/nahsor.cpp"
  #todo
)
set(dependencies
  rclcpp
  rm_interfaces
  rm_util
  rm_cam
  rm_trajectory
  OpenCV
  g2o
  Eigen3
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} g2o::core g2o::solver_eigen)
# Create executable nodes
add_executable(nahsor "src/nahsor_main.cpp")
target_link_libraries(nahsor ${PROJECT_NAME})

# Install launch res directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# # Install executable nodes
install(TARGETS nahsor
  DESTINATION lib/${PROJECT_NAME})

install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)  
# export nahsor dependcy
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} eigen3_cmake_module)

rclcpp_components_register_nodes(${PROJECT_NAME} "nahsor::NahsorNode")
#test
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
