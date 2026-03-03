#!/bin/bash

# 1. 경로 설정
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# 2. 기본값 설정
TARGET="nvidia" # 기본값은 nvidia(x86)
USER_NAME=$(whoami)
USER_UID=$(id -u)
USER_GID=$(id -g)

# 3. 입력 인자(Argument) 처리
usage() {
    echo "Usage: $0 [-t cpu|nvidia|jetpack]"
    echo "  -t: 빌드 타겟 선택 (기본값: nvidia)"
    exit 1
}

while getopts "t:" opt; do
    case $opt in
        t) TARGET=$OPTARG ;;
        *) usage ;;
    esac
done

# 4. 아키텍처별 설정 분기
case $TARGET in
    "cpu")
        DOCKERFILE="Dockerfile.cpu"
        IMAGE_TAG="kaair:humble-cpu"
        ;;
    "nvidia")
        DOCKERFILE="Dockerfile.nvidia"
        IMAGE_TAG="kaair:humble-nvidia"
        ;;
    "jetpack")
        DOCKERFILE="Dockerfile.jetpack"
        IMAGE_TAG="kaair:humble-jetpack"
        ;;
    *)
        echo "❌ 알 수 없는 타겟입니다: $TARGET (사용 가능: cpu, nvidia, jetpack)"
        usage
        ;;
esac

echo "🚀 빌드를 시작합니다: [$TARGET] 모드"
echo "📍 Dockerfile: $DOCKERFILE"
echo "🏷️ Image Tag: $IMAGE_TAG"
echo " User Name: $USER_NAME"
echo " UID: $USER_UID"
echo " GID: $USER_GID"


# 5. 빌드 실행
# --build-arg로 UID/GID뿐만 아니라 TARGET 정보도 넘겨줄 수 있습니다.
docker build "$PROJECT_ROOT" \
    -f "$PROJECT_ROOT/docker/$DOCKERFILE" \
    -t "$IMAGE_TAG" \
    --build-arg USERNAME="$USER_NAME" \
    --build-arg USER_UID="$USER_UID" \
    --build-arg USER_GID="$USER_GID" \
    --build-arg TARGET_MODE="$TARGET"

echo "✅ 빌드 완료: $IMAGE_TAG"