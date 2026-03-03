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
        COMPOSE_FILE="docker-compose.cpu.yml"
        ;;
    "nvidia")
        DOCKERFILE="Dockerfile.nvidia"
        IMAGE_TAG="kaair:humble-nvidia"
        COMPOSE_FILE="docker-compose.nvidia.yml"
        ;;
    "jetpack")
        DOCKERFILE="Dockerfile.jetpack"
        IMAGE_TAG="kaair:humble-jetpack"
        COMPOSE_FILE="docker-compose.jetpack.yml"
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
docker build "$PROJECT_ROOT" \
    -f "$PROJECT_ROOT/docker/$DOCKERFILE" \
    -t "$IMAGE_TAG" \
    --build-arg USERNAME="$USER_NAME" \
    --build-arg USER_UID="$USER_UID" \
    --build-arg USER_GID="$USER_GID" \
    --build-arg TARGET_MODE="$TARGET"

# 빌드 성공 여부 확인 후 복사 진행
if [ $? -eq 0 ]; then
    echo "✅ 빌드 완료: $IMAGE_TAG"
    
    # 6. Docker Compose 파일 복사
    # 프로젝트 루트에 있는 docker-compose.{target}.yml을 docker-compose.yml로 복사
    if [ -f "$PROJECT_ROOT/$COMPOSE_FILE" ]; then
        cp "$PROJECT_ROOT/$COMPOSE_FILE" "$PROJECT_ROOT/docker-compose.yml"
        echo "📂 설정 복사 완료: $COMPOSE_FILE -> docker-compose.yml"
    else
        echo "⚠️ 경고: $COMPOSE_FILE 파일을 찾을 수 없어 복사를 건너뜁니다."
    fi
else
    echo "❌ 빌드 실패: 이미지를 생성하지 못했습니다."
    exit 1
fi