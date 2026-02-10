#!/bin/bash

# 1. ROS 환경 소싱
source /opt/ros/${ROS_DISTRO}/setup.bash

# 2. 실행 명령어 결정
# VIRTUAL_ENV 변수가 비어있지 않으면 venv 상태로 간주
if [ -n "$VIRTUAL_ENV" ]; then
    COLCON_CMD="python -m colcon"
    echo "--- Detected Virtual Environment: Using 'python -m colcon' ---"
else
    COLCON_CMD="colcon"
    echo "--- System Environment: Using 'colcon' directly ---"
fi

# 3. 패키지 목록 읽기
PKGS=$(cat .build_config/pkgs-control.txt)

# 4. 빌드 실행
$COLCON_CMD build \
    --symlink-install \
    --metas .build_config/colcon_build.meta \
    --packages-up-to $PKGS \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash