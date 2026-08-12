#!/bin/bash
set -e

# -----------------------------------------------------------------------------
# 1. 경로 설정
# -----------------------------------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# -----------------------------------------------------------------------------
# 2. 인자 파싱 (ROS distro 지정, 기본값 jazzy)
# -----------------------------------------------------------------------------
ROS_DISTRO="jazzy"
SUPPORTED_DISTROS=("jazzy" "humble")

usage() {
    echo "Usage: $0 [-t ros_distro]"
    echo "  -t <distro>   빌드할 ROS 2 배포판 (기본값: jazzy)"
    echo "                지원: ${SUPPORTED_DISTROS[*]}"
    echo
    echo "Example: $0 -t humble"
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

# -----------------------------------------------------------------------------
# 3. 기본 설정
# -----------------------------------------------------------------------------
DOCKERFILE="Dockerfile"
IMAGE_TAG="kaair-moveit:latest"

# 호스트 사용자 정보
USER_UID=$(id -u)
USER_GID=$(id -g)

# -----------------------------------------------------------------------------
# 4. 빌드 정보 출력
# -----------------------------------------------------------------------------
echo "============================================================"
echo "🚀 Docker Build"
echo "============================================================"
echo "Dockerfile  : $DOCKERFILE"
echo "ROS Distro  : $ROS_DISTRO"
echo "Image Tag   : $IMAGE_TAG"
echo "UID         : $USER_UID"
echo "GID         : $USER_GID"
echo "============================================================"


# -----------------------------------------------------------------------------
# 5. Docker Build
# -----------------------------------------------------------------------------
docker build \
    "$PROJECT_ROOT" \
    -f "$PROJECT_ROOT/docker/$DOCKERFILE" \
    -t "$IMAGE_TAG" \
    --build-arg ROS_DISTRO="$ROS_DISTRO" \
    --build-arg USER_UID="$USER_UID" \
    --build-arg USER_GID="$USER_GID"

echo
echo "✅ Build completed successfully!"
echo "Image: $IMAGE_TAG"