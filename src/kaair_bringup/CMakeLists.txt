cmake_minimum_required(VERSION 3.8)
project(kaair_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 설치 규칙: config, launch, rviz 폴더 전체를 share/PROJECT_NAME 디렉토리로 복사
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ---- C++ 노드: depth_resize_node ----
# (기존 depth_resizer_node.py 를 대체)
add_executable(depth_resizer src/depth_resize_node.cpp)
ament_target_dependencies(depth_resizer
  rclcpp
  sensor_msgs
  image_transport
)

# ---- C++ 노드: depth_hole_fill_node ----
# hand camera depth(image_rect_raw)의 0(무효) 픽셀을 median filter로 채워
# 홀을 메운다. 해상도는 그대로 유지.
add_executable(depth_hole_filler src/depth_hole_fill_node.cpp)
ament_target_dependencies(depth_hole_filler
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)
target_link_libraries(depth_hole_filler ${OpenCV_LIBRARIES})

install(TARGETS
  depth_resizer
  depth_hole_filler
  DESTINATION lib/${PROJECT_NAME}
)

# 1. 경로와 이름을 공백이나 줄바꿈으로 나열 (세미콜론 제외)
set(PYTHON_NODES
  "kaair_bringup/server_worker/location_server.py"               "location_server"
  "kaair_bringup/server_worker/robot_pose_publisher.py"          "robot_pose_publisher"
  "kaair_bringup/server_worker/object_marker_server.py"               "object_marker_server"
  "kaair_bringup/server_worker/depth_pointcloud_publisher.py"         "depth_pointcloud_publisher"
  "kaair_bringup/server_worker/xarm_bridge.py"                   "xarm_bridge"
  "kaair_bringup/motion_worker/lift_move_action_server.py"       "lift_move_action_server"
  "kaair_bringup/motion_worker/arm_move_action_server.py"        "arm_move_action_server"
  "kaair_bringup/motion_worker/controller_mode_switcher.py"      "controller_mode_switcher"
  "kaair_bringup/motion_worker/head_move_server.py"              "head_move_server"
  "kaair_bringup/master_worker/master_dxl_hw.py"                "master_dxl_hw"
  "kaair_bringup/master_worker/master_controller.py"                "master_controller"
  "kaair_bringup/master_worker/3d_master_controller.py"           "3d_master_controller"
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
