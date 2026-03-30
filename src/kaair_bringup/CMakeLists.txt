cmake_minimum_required(VERSION 3.8)
project(kaair_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kaair_description REQUIRED)
find_package(kaair_controller REQUIRED)
find_package(kaair_moveit_config REQUIRED)

# 설치 규칙: config, launch, rviz 폴더 전체를 share/PROJECT_NAME 디렉토리로 복사
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 새로운 파이썬 노드가 생길때마다 파이썬 스크립드 경로:리네임 노드 이름 정의
set(PYTHON_NODES
  "kaair_bringup/server/location_server.py;location_manager"
)


foreach(NODE_INFO "${PYTHON_NODES}")
  # 리스트 요소를 분리 (0번: 경로, 1번: 새 이름)
  list(GET NODE_INFO 0 NODE_PATH)
  list(GET NODE_INFO 1 NODE_NAME)

  install(PROGRAMS ${NODE_PATH}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${NODE_NAME}
  )
endforeach()



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
