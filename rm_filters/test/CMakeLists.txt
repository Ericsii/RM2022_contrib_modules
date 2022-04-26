find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(kf_test kf_test.cpp)
target_link_libraries(kf_test ${PROJECT_NAME})