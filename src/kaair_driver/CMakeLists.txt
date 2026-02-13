cmake_minimum_required(VERSION 3.8)
project(kaair_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(dynamixel_sdk REQUIRED)

# 2. 라이브러리 생성 (공유 라이브러리 SHARED 필수)
add_library(kaair_driver SHARED
  src/head/head_hw_interface.cpp
)

# 3. 인클루드 경로 설정
target_include_directories(kaair_driver PRIVATE
  include
)

ament_target_dependencies(kaair_driver
  hardware_interface
  pluginlib
  rclcpp
  dynamixel_sdk
)

# 플러그인 등록 XML 파일 (파일명 확인 필수)
pluginlib_export_plugin_description_file(
  hardware_interface 
  hw_plugins.xml
)

install(TARGETS kaair_driver
  DESTINATION lib)

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
