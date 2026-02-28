#!/bin/bash

# 1. 스크립트 파일이 위치한 실제 경로를 구함 (pkg/script)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# 2. 프로젝트 루트 경로 설정 (script 폴더의 상위 폴더인 pkg)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# 3. 이미지 이름 설정
IMAGE_NAME="kaair:humble-cuda"

echo "--- Build Info ---"
echo "Project Root: $PROJECT_ROOT"
echo "Dockerfile:   $PROJECT_ROOT/docker/Dockerfile.nvidia"
echo "Image Name:   $IMAGE_NAME"
echo "------------------"

# 4. 빌드 실행 (Context를 PROJECT_ROOT로 고정)
docker build "$PROJECT_ROOT" \
    -f "$PROJECT_ROOT/docker/Dockerfile.nvidia" \
    -t "$IMAGE_NAME"