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

# 1. 경로와 이름을 공백이나 줄바꿈으로 나열 (세미콜론 제외)
set(PYTHON_NODES
  "kaair_bringup/server/location_server.py" "location_manager"
  "kaair_bringup/server/xarm_bridge.py"     "xarm_manager"
  # "추가경로" "추가이름"
)

# 2. 2개씩(NODE_PATH, NODE_NAME) 세트로 가져오기
# foreach(variable IN ITEMS list) 구문을 사용합니다.
list(LENGTH PYTHON_NODES NODE_COUNT)

math(EXPR STOP_INDEX "${NODE_COUNT} - 1")
foreach(INDEX RANGE 0 ${STOP_INDEX} 2)
  # 현재 인덱스(경로)와 다음 인덱스(이름) 가져오기
  list(GET PYTHON_NODES ${INDEX} NODE_PATH)
  
  math(EXPR NAME_INDEX "${INDEX} + 1")
  list(GET PYTHON_NODES ${NAME_INDEX} NODE_NAME)

  # 설치 실행
  install(PROGRAMS ${NODE_PATH}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${NODE_NAME}
  )
  
  message(STATUS "Installing node: ${NODE_PATH} as ${NODE_NAME}")
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
