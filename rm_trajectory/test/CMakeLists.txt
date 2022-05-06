find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory ${PROJECT_NAME})
