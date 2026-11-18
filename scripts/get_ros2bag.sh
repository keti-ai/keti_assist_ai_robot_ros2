
#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# 1. 경로 설정
# -----------------------------------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

BAG_NAME="r2b_cafe"
BAG_DIR="$PROJECT_ROOT/$BAG_NAME"

# NVIDIA R2B Dataset 2023
BASE_URL="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/r2bdataset2023/versions/1/files"

# -----------------------------------------------------------------------------
# 2. 의존성 확인
# -----------------------------------------------------------------------------
if ! command -v curl >/dev/null 2>&1; then
    echo "❌ curl이 필요합니다: sudo apt install curl"
    exit 1
fi

# -----------------------------------------------------------------------------
# 3. 다운로드 폴더 생성
# -----------------------------------------------------------------------------
mkdir -p "$BAG_DIR"

echo "============================================================"
echo "📦 Download ROS2 Bag: $BAG_NAME"
echo "============================================================"
echo "Project root : $PROJECT_ROOT"
echo "Download dir : $BAG_DIR"
echo "Dataset      : NVIDIA R2B Dataset 2023"
echo "============================================================"

# -----------------------------------------------------------------------------
# 4. 파일 다운로드
# -----------------------------------------------------------------------------
download_file() {
    local filename="$1"
    local url="$BASE_URL/$BAG_NAME/$filename"
    local output="$BAG_DIR/$filename"

    echo
    echo "⬇ Downloading: $filename"

    curl \
        --fail \
        --location \
        --retry 5 \
        --retry-delay 3 \
        --connect-timeout 30 \
        --continue-at - \
        --output "$output" \
        "$url"

    echo "✅ Downloaded: $output"
}

download_file "metadata.yaml"
download_file "${BAG_NAME}_0.db3"

# -----------------------------------------------------------------------------
# 5. 완료
# -----------------------------------------------------------------------------
echo
echo "============================================================"
echo "✅ ROS2 Bag download completed"
echo "============================================================"

du -h "$BAG_DIR"/*

echo
echo "Playback:"
echo "  ros2 bag play \"$BAG_DIR\""
echo
echo "Bag info:"
echo "  ros2 bag info \"$BAG_DIR\""
