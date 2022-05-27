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
    src/ballistic_trajectory/database_solver.cpp
    src/ballistic_trajectory/gravity_solver.cpp
    src/ballistic_trajectory/iterative_trajectory_tool.cpp
    src/ballistic_trajectory/transform_tool.cpp
    src/ballistic_trajectory/gravity_nofriction_solver.cpp
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
  add_subdirectory(test)
endif()

ament_package() 

