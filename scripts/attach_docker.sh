#!/bin/bash
set -e

# -----------------------------------------------------------------------------
# 1. 인자 파싱 (컨테이너 이름 직접 지정하고 싶을 때만 -c 사용)
# -----------------------------------------------------------------------------
NAME_PREFIX="keti_ros2_container_"
CONTAINER_NAME=""

usage() {
    echo "Usage: $0 [-c container_name]"
    echo "  -c <name>   접속할 컨테이너 이름 (미지정 시 실행 중인 ${NAME_PREFIX}* 컨테이너를 자동 탐색)"
    echo
    echo "Example: $0"
    echo "         $0 -c keti_ros2_container_humble"
    exit 1
}

while getopts ":c:h" opt; do
    case "$opt" in
        c) CONTAINER_NAME="$OPTARG" ;;
        h) usage ;;
        \?) echo "❌ Unknown option: -$OPTARG"; usage ;;
        :) echo "❌ Option -$OPTARG requires an argument"; usage ;;
    esac
done

# -----------------------------------------------------------------------------
# 2. 컨테이너 이름 자동 탐색 (humble/jazzy에 따라 이름이 다름: keti_ros2_container_<distro>)
# -----------------------------------------------------------------------------
if [ -z "$CONTAINER_NAME" ]; then
    MATCHES=$(docker ps --format '{{.Names}}' | grep "^${NAME_PREFIX}" || true)
    MATCH_COUNT=$(echo "$MATCHES" | grep -c . || true)

    if [ "$MATCH_COUNT" -eq 0 ]; then
        echo "❌ 실행 중인 ${NAME_PREFIX}* 컨테이너가 없습니다."
        echo "   먼저 실행하세요: bash scripts/run_docker.sh -p <cpu|nvidia|jetpack>"
        exit 1
    elif [ "$MATCH_COUNT" -gt 1 ]; then
        echo "❌ 실행 중인 컨테이너가 여러 개입니다. -c 로 지정하세요:"
        echo "$MATCHES" | sed 's/^/   - /'
        exit 1
    fi

    CONTAINER_NAME="$MATCHES"
fi

# -----------------------------------------------------------------------------
# 3. attach
# -----------------------------------------------------------------------------
echo "🔗 Attaching to $CONTAINER_NAME ..."
docker exec -it "$CONTAINER_NAME" /bin/bash
