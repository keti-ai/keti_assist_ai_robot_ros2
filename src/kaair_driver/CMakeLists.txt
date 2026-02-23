cmake_minimum_required(VERSION 3.8)
project(kaair_driver)

# C++ 표준 명시 (현대적 C++ 기능을 위해 필수)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# 1. 의존성 패키지 로드
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(dynamixel_sdk REQUIRED)

# -------------------------------------------------------------------
# 2. 공통 하드웨어 로직 라이브러리 (확장성의 핵심)
# -------------------------------------------------------------------
# 하드웨어 인터페이스와 테스트 실행 파일이 공통으로 사용할 로직을 분리합니다.
add_library(dxl_hw_lib SHARED
  src/head/dxl_hw.cpp  # 실제 다이나믹셀 SDK를 다루는 클래스 구현부
)

target_include_directories(dxl_hw_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dxl_hw_lib
  dynamixel_sdk
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

# -------------------------------------------------------------------
# 3. ROS 2 Control Hardware Interface 플러그인
# -------------------------------------------------------------------
add_library(kaair_driver SHARED
  src/head/head_hw_interface.cpp
)

target_include_directories(kaair_driver PRIVATE include)

# 공통 라이브러리(dxl_hw_lib)를 링크합니다.
target_link_libraries(kaair_driver dxl_hw_lib)

ament_target_dependencies(kaair_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# -------------------------------------------------------------------
# 4. 테스트/디버깅용 실행 파일 (현재 연습 중인 부분)
# -------------------------------------------------------------------
add_executable(dxl_test_node src/head/test_dxl.cpp)

# [수정] 실행 파일이 include 폴더를 뒤질 수 있도록 경로를 직접 추가합니다.
target_include_directories(dxl_test_node PRIVATE 
  include
)

# [수정] 라이브러리 링크 및 의존성 추가
target_link_libraries(dxl_test_node dxl_hw_lib)
ament_target_dependencies(dxl_test_node
  dynamixel_sdk
  rclcpp
)

#

# -------------------------------------------------------------------
# 5. 플러그인 등록 및 설치
# -------------------------------------------------------------------
pluginlib_export_plugin_description_file(
  hardware_interface 
  hw_plugins.xml
)

# 실행 파일과 라이브러리를 모두 설치
install(TARGETS 
  dxl_hw_lib
  kaair_driver
  dxl_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (다른 패키지에서 참조할 수 있도록)
install(DIRECTORY include/ 
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()