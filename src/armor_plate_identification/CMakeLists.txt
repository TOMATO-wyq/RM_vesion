cmake_minimum_required(VERSION 3.8)
project(armor_plate_identification)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================
# 打包
# ======================
set(  ArmorPlateIdentifcation
  src/ArmorPlateIdentifcation.cpp
  src/CameraDriver.cpp
  src/PoseSolver.cpp
  src/Detector.cpp
  src/DebugIdentifaction.cpp
  src/NumberClassifier.cpp
)
set( Test
  src/Test.cpp
  src/DebugIdentifaction.cpp
  src/PoseSolver.cpp
  src/Detector.cpp
  src/NumberClassifier.cpp
)
# ======================
# 依赖
# ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(armor_plate_interfaces REQUIRED)

# ======================
# 库文件
# ======================
add_executable(ArmorPlateIdentifcation ${ArmorPlateIdentifcation})
add_executable(Test ${Test})

# ======================
# 大恒 SDK 路径
# ======================
set(GALAXY_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_parties/GalaxySDK")
# ======================
# 迈德 SDK 路径
# ======================
set(MINDVISION_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_parties/mvsdk")

# ======================
# include 路径
# ======================
target_include_directories(ArmorPlateIdentifcation PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${GALAXY_SDK_ROOT}/include
${MINDVISION_SDK_ROOT}/include
)
target_include_directories(Test PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_include_directories(ArmorPlateIdentifcation PUBLIC
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
target_include_directories(Test PUBLIC
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
target_compile_features(ArmorPlateIdentifcation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(Test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ======================
# 链接库
# ======================
target_link_directories(ArmorPlateIdentifcation PUBLIC
  ${GALAXY_SDK_ROOT}/lib/amd64
  ${MINDVISION_SDK_ROOT}/lib/amd64
)
target_link_libraries(ArmorPlateIdentifcation
${OpenCV_LIBS}
gxiapi
MVSDK
)
target_link_libraries(Test
${OpenCV_LIBS}
)
# ======================
# 依赖关系
# ======================
ament_target_dependencies(
  ArmorPlateIdentifcation
  "rclcpp"
  "OpenCV"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "tf2_ros"
  "image_transport"
  "camera_info_manager"
  "armor_plate_interfaces"
)
ament_target_dependencies(
  Test
  "rclcpp"
  "OpenCV"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "tf2_ros"
  "armor_plate_interfaces"
)
  
# ======================
# 安装第三方动态库
# ======================
file(GLOB GALAXY_LIBS "${GALAXY_SDK_ROOT}/lib/amd64/*")
install(FILES ${GALAXY_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)
file(GLOB MINDVISION_LIBS "${MINDVISION_SDK_ROOT}/lib/amd64/*")
install(FILES ${MINDVISION_LIBS}
  DESTINATION lib/${PROJECT_NAME}
)

# ======================
# 环境钩子：自动配置 LD_LIBRARY_PATH / GENICAM_GENTL64_PATH
# ======================
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/third_parties_lib_path.sh.in")

# ======================
# 安装
# ======================
install(TARGETS 
  ArmorPlateIdentifcation Test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY
  config video launch model
  DESTINATION share/${PROJECT_NAME}
)


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

ament_package()
