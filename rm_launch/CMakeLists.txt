cmake_minimum_required(VERSION 3.8)
project(rm_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rm_base REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rm_auto_aim REQUIRED)
find_package(rm_entity_cam REQUIRED)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
