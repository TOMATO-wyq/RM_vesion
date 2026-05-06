cmake_minimum_required(VERSION 3.8)
project(armor_plate_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(armor_plate_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(armor_plate_tracker_node
  src/ArmorPlateTracker.cpp
  src/MyExtendedKalmanFilter.cpp
  src/Tracker.cpp
  src/DebugTracker.cpp
)

target_include_directories(armor_plate_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CAMERA_SDK_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(armor_plate_tracker_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  armor_plate_tracker_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "Eigen3"
  "armor_plate_interfaces"
  "image_transport"
  "camera_info_manager"
  "tf2_ros"
  "visualization_msgs"
)

target_link_libraries(armor_plate_tracker_node
  ${OpenCV_LIBS}
)

install(TARGETS armor_plate_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
