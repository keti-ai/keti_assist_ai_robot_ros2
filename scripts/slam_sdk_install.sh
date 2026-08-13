#!/bin/bash
set -e

# 1. 경로 정의
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$PROJECT_ROOT/ros2/slam_ws/src"

SDK_VERSION="5.1.1-rtm"
REFERER="https://download-en.slamtec.com/"
USER_AGENT="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0.0.0 Safari/537.36"

# 2. 아키텍처 감지
ARCH=$(uname -m)
if [[ "$ARCH" == "x86_64" ]]; then
  SDK_PRODUCT="slamware-ros2-sdk_x86_64_gcc11"
elif [[ "$ARCH" == "aarch64" ]]; then
  SDK_PRODUCT="slamware-ros2-linux-aarch64_gcc11"
else
  echo "❌ 지원하지 않는 아키텍처: $ARCH"
  exit 1
fi

DOWNLOAD_URL="https://download-en.slamtec.com/api/download/${SDK_PRODUCT}/${SDK_VERSION}?lang=netural"

echo "==> [1] 아키텍처 확인: $ARCH"
echo "==> [2] 다운로드 URL: $DOWNLOAD_URL"

# 3. 임시 작업 공간 준비
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT
ARCHIVE_PATH="$WORK_DIR/slamware_ros2_sdk.tar.gz"

# 4. SDK 다운로드
echo "==> [3] SDK 다운로드 중..."
wget --header="Referer: $REFERER" \
     --user-agent="$USER_AGENT" \
     "$DOWNLOAD_URL" \
     -O "$ARCHIVE_PATH"
echo "✅ 다운로드 완료"

# 5. 압축 해제
echo "==> [4] 압축 해제 중..."
tar -xzf "$ARCHIVE_PATH" -C "$WORK_DIR"

EXTRACTED_DIR="$(find "$WORK_DIR" -mindepth 1 -maxdepth 1 -type d)"
EXTRACTED_SRC_DIR="$EXTRACTED_DIR/src"

if [ ! -d "$EXTRACTED_SRC_DIR" ]; then
  echo "❌ 압축 파일 내에서 src 디렉토리를 찾을 수 없습니다."
  exit 1
fi

# 6. src 내용물을 slam_ws/src 에 배치
echo "==> [5] $SRC_DIR 에 설치 중..."
mkdir -p "$SRC_DIR"
for item in "$EXTRACTED_SRC_DIR"/*; do
  target="$SRC_DIR/$(basename "$item")"
  rm -rf "$target"
  mv "$item" "$target"
done

# 7. ROS 2 Jazzy 호환성 패치
# Jazzy에서는 deprecated된 tf2_geometry_msgs.h shim이 제거되어 .hpp만 존재함
echo "==> [6] 호환성 패치 적용 중..."
MSG_CONVERT_H="$SRC_DIR/slamware_ros_sdk/src/server/msg_convert.h"
if [ -f "$MSG_CONVERT_H" ]; then
  sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h>|tf2_geometry_msgs/tf2_geometry_msgs.hpp>|' "$MSG_CONVERT_H"
  echo "✅ 패치 완료: $MSG_CONVERT_H (tf2_geometry_msgs.h -> .hpp)"
else
  echo "⚠️ 패치 대상 파일을 찾을 수 없습니다: $MSG_CONVERT_H"
fi

echo "✅ 설치 완료: $SRC_DIR"
