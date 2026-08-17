#!/bin/bash
set -e

# -----------------------------------------------------------------------------
# 1. 경로 설정
# -----------------------------------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
ROS2_ROOT="$PROJECT_ROOT/ros2"
XARM_WS="$ROS2_ROOT/xarm_ws"
CAMERA_WS="$ROS2_ROOT/camera_ws"

# -----------------------------------------------------------------------------
# 2. 인자 파싱 (ROS distro 지정)
#    우선순위: -t 옵션 > ROS_DISTRO 환경변수 > humble
# -----------------------------------------------------------------------------
ROS_DISTRO="${ROS_DISTRO:-humble}"
SUPPORTED_DISTROS=("jazzy" "humble")

usage() {
    echo "Usage: $0 [-t ros_distro]"
    echo "  -t <distro>   xarm_ros2 브랜치로 사용할 ROS 2 배포판 (기본값: ${ROS_DISTRO})"
    echo "                지원: ${SUPPORTED_DISTROS[*]}"
    echo
    echo "Example: $0 -t humble"
    echo "         ROS_DISTRO=jazzy $0"
    exit 1
}

while getopts ":t:h" opt; do
    case "$opt" in
        t) ROS_DISTRO="$OPTARG" ;;
        h) usage ;;
        \?) echo "❌ Unknown option: -$OPTARG"; usage ;;
        :) echo "❌ Option -$OPTARG requires an argument"; usage ;;
    esac
done

if [[ ! " ${SUPPORTED_DISTROS[*]} " =~ " ${ROS_DISTRO} " ]]; then
    echo "❌ Unsupported ROS_DISTRO: ${ROS_DISTRO} (지원: ${SUPPORTED_DISTROS[*]})"
    exit 1
fi

REPOS_FILE="$PROJECT_ROOT/third_party.${ROS_DISTRO}.repos"

if [ ! -f "$REPOS_FILE" ]; then
    echo "❌ repos 파일이 없습니다: $REPOS_FILE"
    exit 1
fi

if ! command -v vcs >/dev/null 2>&1; then
    echo "❌ vcstool이 필요합니다. 설치: sudo apt install vcstool"
    exit 1
fi

# -----------------------------------------------------------------------------
# 3. import
#    third_party.*.repos 경로는 저장소 루트 기준:
#      ros2/xarm_ws/src/xarm_ros2
#      ros2/camera_ws/src/OrbbecSDK_ROS2
#      ros2/camera_ws/src/Realsense_ROS2
# -----------------------------------------------------------------------------
mkdir -p "$XARM_WS/src" "$CAMERA_WS/src"

echo "============================================================"
echo "📦 Import third-party packages"
echo "============================================================"
echo "ROS Distro  : $ROS_DISTRO"
echo "Repos file  : $REPOS_FILE"
echo "xarm        : $XARM_WS/src/xarm_ros2"
echo "camera      : $CAMERA_WS/src/{OrbbecSDK_ROS2,Realsense_ROS2}"
echo "============================================================"

vcs import --recursive --skip-existing "$PROJECT_ROOT" < "$REPOS_FILE"

echo
echo "✅ Import completed (xarm_ros2 branch: $ROS_DISTRO)"
echo "   colcon build 는 각 워크스페이스에서 실행하세요:"
echo "     cd $XARM_WS && colcon build --symlink-install"
echo "     cd $CAMERA_WS && colcon build --symlink-install"
