cmake_minimum_required(VERSION 3.8)
project(kaair_driver)

# C++ 표준 명시 (현대적 C++ 기능을 위해 필수)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
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
find_package(serial REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(MODBUS REQUIRED libmodbus)

# -------------------------------------------------------------------
# 2. 공통 하드웨어 로직 라이브러리 (확장성의 핵심)
# -------------------------------------------------------------------
# [기존] Head (다이나믹셀) 로직
add_library(dxl_hw_lib STATIC
  src/head/dxl_hw.cpp
)
target_include_directories(dxl_hw_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dxl_hw_lib
  dynamixel_sdk
)

# Lift (MD400) 로직
add_library(md485_hw_lib STATIC
  src/lift/md485_hw.cpp
)

target_include_directories(md485_hw_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(md485_hw_lib
  serial
)

add_library(koras_hw_lib STATIC
  src/tool/koras_hw.cpp
)
target_include_directories(koras_hw_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)
target_link_libraries(koras_hw_lib
  ${MODBUS_LIBRARIES}
)

# -------------------------------------------------------------------
# 3. ROS 2 Control Hardware Interface 플러그인
# -------------------------------------------------------------------
add_library(kaair_driver SHARED
  src/head/head_hw_interface.cpp
  src/lift/lift_hw_interface.cpp
  src/tool/tool_hw_interface.cpp
)

target_include_directories(kaair_driver PRIVATE include)

# 공통 라이브러리(dxl_hw_lib)를 링크합니다.
target_link_libraries(kaair_driver 
  dxl_hw_lib 
  md485_hw_lib 
  koras_hw_lib
  ${MODBUS_LIBRARIES}  # 👈 여기에 추가!
)

ament_target_dependencies(kaair_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
  dynamixel_sdk
)

# -------------------------------------------------------------------
# 4. 테스트/디버깅용 실행 파일
# -------------------------------------------------------------------
# Head 테스트용
add_executable(dxl_test_node src/head/test_dxl.cpp)
target_include_directories(dxl_test_node PRIVATE include)
target_link_libraries(dxl_test_node dxl_hw_lib)
ament_target_dependencies(dxl_test_node)

# Lift 테스트용 실행 파일 (lift_test_node)
add_executable(lift_test_node src/lift/test_lift.cpp)
target_include_directories(lift_test_node PRIVATE include)
target_link_libraries(lift_test_node md485_hw_lib)
ament_target_dependencies(lift_test_node)

# Lift 단독 호밍 초기화 노드
add_executable(lift_initializer src/lift/lift_initializer.cpp)
target_include_directories(lift_initializer PRIVATE include)
target_link_libraries(lift_initializer md485_hw_lib) # 통신 라이브러리 연결

# Koras 그리퍼 테스트용 실행 파일
add_executable(koras_test_node src/tool/test_koras.cpp)
target_include_directories(koras_test_node PRIVATE include)
target_link_libraries(koras_test_node koras_hw_lib ${MODBUS_LIBRARIES})
ament_target_dependencies(koras_test_node)


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
  md485_hw_lib
  koras_hw_lib
  kaair_driver
  dxl_test_node
  lift_test_node
  lift_initializer
  koras_test_node
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