#!/bin/bash

# 1. 경로 정의
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_DIR="$PROJECT_ROOT/src/third_party/slamware_sdk"
SDK_REPO="https://github.com/keti-ai/slamware_sdk.git"

# 2. 아키텍처 감지
ARCH=$(uname -m)
if [[ "$ARCH" == "x86_64" ]]; then
  SDK_BRANCH="amd64"
elif [[ "$ARCH" == "aarch64" ]]; then
  SDK_BRANCH="aarch64"
else
  echo "❌ 지원하지 않는 아키텍처: $ARCH"
  exit 1
fi

echo "==> [1] 아키텍처 확인: $ARCH (Target Branch: $SDK_BRANCH)"

# 3. 설치 또는 업데이트 로직
if [ ! -d "$TARGET_DIR/.git" ]; then
    echo "==> [2] SDK가 없습니다. 새로 설치합니다..."
    rm -rf "$TARGET_DIR" # 혹시 남아있을 빈 폴더 제거
    git clone -b "$SDK_BRANCH" --depth 1 "$SDK_REPO" "$TARGET_DIR"
    echo "✅ 설치 완료"
else
    echo "==> [2] 이미 SDK가 존재합니다. 업데이트를 확인합니다..."
    cd "$TARGET_DIR"
    
    # 현재 브랜치가 맞는지 확인하고 업데이트
    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    if [ "$CURRENT_BRANCH" == "$SDK_BRANCH" ]; then
        git pull origin "$SDK_BRANCH"
        echo "✅ 업데이트 완료"
    else
        echo "⚠️ 브랜치가 다릅니다 ($CURRENT_BRANCH -> $SDK_BRANCH). 다시 설치합니다."
        cd "$PROJECT_ROOT"
        rm -rf "$TARGET_DIR"
        git clone -b "$SDK_BRANCH" --depth 1 "$SDK_REPO" "$TARGET_DIR"
        echo "✅ 재설치 완료"
    fi
fi