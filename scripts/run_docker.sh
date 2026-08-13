#!/bin/bash
set -e

# -----------------------------------------------------------------------------
# 1. 경로 설정
# -----------------------------------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# -----------------------------------------------------------------------------
# 2. 인자 파싱 (플랫폼 지정, 기본값 cpu)
#    ROS_DISTRO / RMW_IMPLEMENTATION 은 -d/-r 로 override (미지정 시 .env 기본값 사용)
# -----------------------------------------------------------------------------
PLATFORM="cpu"
SUPPORTED_PLATFORMS=("cpu" "nvidia" "jetpack")
DISTRO_OVERRIDE=""
RMW_OVERRIDE=""

usage() {
    echo "Usage: $0 [-p platform] [-d ros_distro] [-r rmw_implementation]"
    echo "  -p <platform>   실행할 플랫폼 (기본값: cpu)"
    echo "                  지원: ${SUPPORTED_PLATFORMS[*]}"
    echo "  -d <distro>     ROS_DISTRO override (미지정 시 .env 값 사용)"
    echo "  -r <rmw>        RMW_IMPLEMENTATION override (미지정 시 .env 값 사용)"
    echo
    echo "Example: $0 -p nvidia"
    echo "         $0 -p cpu -d humble -r rmw_fastrtps_cpp"
    exit 1
}

while getopts ":p:d:r:h" opt; do
    case "$opt" in
        p) PLATFORM="$OPTARG" ;;
        d) DISTRO_OVERRIDE="$OPTARG" ;;
        r) RMW_OVERRIDE="$OPTARG" ;;
        h) usage ;;
        \?) echo "❌ Unknown option: -$OPTARG"; usage ;;
        :) echo "❌ Option -$OPTARG requires an argument"; usage ;;
    esac
done

if [[ ! " ${SUPPORTED_PLATFORMS[*]} " =~ " ${PLATFORM} " ]]; then
    echo "❌ Unsupported platform: ${PLATFORM} (지원: ${SUPPORTED_PLATFORMS[*]})"
    exit 1
fi

COMPOSE_FILE="$PROJECT_ROOT/docker-compose.${PLATFORM}.yml"

if [ ! -f "$COMPOSE_FILE" ]; then
    echo "❌ compose 파일이 없습니다: $COMPOSE_FILE"
    exit 1
fi

[ -n "$DISTRO_OVERRIDE" ] && export ROS_DISTRO="$DISTRO_OVERRIDE"
[ -n "$RMW_OVERRIDE" ] && export RMW_IMPLEMENTATION="$RMW_OVERRIDE"

# -----------------------------------------------------------------------------
# 3. 실행 정보 출력
# -----------------------------------------------------------------------------
echo "============================================================"
echo "🚀 Docker Compose Up"
echo "============================================================"
echo "Platform    : $PLATFORM"
echo "Compose file: $(basename "$COMPOSE_FILE")"
echo "ROS Distro  : ${ROS_DISTRO:-(.env default)}"
echo "RMW         : ${RMW_IMPLEMENTATION:-(.env default)}"
echo "============================================================"

# -----------------------------------------------------------------------------
# 4. docker compose up
# -----------------------------------------------------------------------------
(cd "$PROJECT_ROOT" && docker compose -f "$COMPOSE_FILE" up -d)

echo
echo "✅ Container started"
echo "   attach: bash scripts/attach_docker.sh"
