cmake_minimum_required(VERSION 3.8)
project(rm_sentry)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DRM_DEBUG_MODE")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rm_cam REQUIRED)
find_package(rm_util REQUIRED)
find_package(rm_filters REQUIRED)
find_package(rm_trajectory REQUIRED)
find_package(rm_auto_aim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
    #OpenVINO推理引擎的头文件
    /home/scurm/intel/openvino_2021/deployment_tools/inference_engine/include/
    /home/scurm/intel/openvino_2021/deployment_tools/ngraph/include/
)
set(InferenceEngine_DIR "/home/scurm/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)
set(ngraph_DIR "/home/scurm/intel/openvino_2021/deployment_tools/ngraph")
find_package(ngraph REQUIRED)
set(ngraph_LIBRARIES "/home/scurm/intel/openvino_2021/deployment_tools/ngraph/lib/libngraph.so")
set(ngraph_INCLUDE_DIRS "/home/scurm/intel/openvino_2021/deployment_tools/ngraph/include/")
# build shared lib
add_library(${PROJECT_NAME} SHARED "src/sentry_node.cpp" "src/sentry_algo.cpp" "src/sentry_detector"  "src/opencv_detector.cpp" )

set(dependencies
  OpenCV
  rm_interfaces
  rm_cam
  rm_util
  rm_filters
  rm_trajectory
  rm_auto_aim
  rclcpp
  rclcpp_components
  Eigen3
  
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# build executable
add_executable(sentry_node "src/sentry_main.cpp")
target_link_libraries(sentry_node ${PROJECT_NAME} ${OpenCV_LIBS}
                      ${InferenceEngine_LIBRARIES}
                      ${ngraph_LIBRARIES})

rclcpp_components_register_nodes(${PROJECT_NAME} "rm_sentry::SentryNode")

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

install(TARGETS sentry_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch resource config directory

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
