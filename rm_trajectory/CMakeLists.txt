cmake_minimum_required(VERSION 3.8)
project(rm_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include
include_directories(include)

#find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

#create rm_trajectory lib
add_library(${PROJECT_NAME} SHARED
    src/ballistic_trajectory/trajectory_pitch.cpp
    src/ballistic_trajectory/trajectory_tool.cpp
    src/ballistic_trajectory/trajectory_gaf.cpp
    src/ballistic_trajectory/trajectory_gimbal.cpp
    src/ballistic_trajectory/trajectory_gravity.cpp
)
set(dependencies
  rclcpp
  Eigen3
  eigen3_cmake_module
  geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#install include directories
install(DIRECTORY include/
  DESTINATION include
)

#install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#export rmoss_projectile_motion lib
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 


