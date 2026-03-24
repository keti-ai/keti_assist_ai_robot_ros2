cmake_minimum_required(VERSION 3.22)
project(kaair_moveit_config)

find_package(ament_cmake REQUIRED)

# 1. 파이썬 모듈 폴더 설치 (utils.py가 포함된 kaair_moveit_config 폴더)
# 이 폴더가 설치되어야 런치 파일에서 'import kaair_moveit_config.utils'가 가능합니다.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/kaair_moveit_config")
  install(
    DIRECTORY kaair_moveit_config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 2. launch 디렉토리 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

# 3. config 디렉토리 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 4. .setup_assistant 설정 파일 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(
    FILES .setup_assistant 
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ament_package()는 반드시 모든 install 명령 뒤에 위치해야 합니다.
ament_package()