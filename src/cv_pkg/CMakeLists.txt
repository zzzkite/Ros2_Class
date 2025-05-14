cmake_minimum_required(VERSION 3.8)
project(cv_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(cv_image src/cv_image.cpp) # 参数：生成可执行文件名+执行.cpp的文件
add_executable(cv_hsv src/cv_hsv.cpp)
add_executable(cv_follow src/cv_follow.cpp)
add_executable(cv_face_detect src/cv_face_detect.cpp)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(cv_bridge REQUIRED) 
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_target_dependencies(cv_image rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(cv_hsv rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(cv_follow rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(cv_face_detect rclcpp sensor_msgs cv_bridge OpenCV)

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

install(TARGETS cv_image cv_hsv cv_follow cv_face_detect
DESTINATION lib/${PROJECT_NAME})

ament_package()
