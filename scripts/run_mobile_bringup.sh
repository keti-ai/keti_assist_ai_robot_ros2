#!/bin/bash
set -e

# -----------------------------------------------------------------------------
# Slamware 관련 노드를 도메인 35(SLAM_DOMAIN)에서 격리 실행하고,
# slamtec_bridge_node 가 도메인 35 <-> 타겟 도메인(TARGET_DOMAIN, 기본값은
# 컨테이너의 현재 ROS_DOMAIN_ID)을 이어준다.
#
#   - slamware_ros_sdk_server_node : SLAM_DOMAIN 전용
#   - slamtec_bridge_node          : SLAM_DOMAIN <-> TARGET_DOMAIN 브릿지
#                                     (slamware_ros_sdk 메시지 import 를 위해
#                                      slam_ws 오버레이가 필요)
#
# slam_ws/install/setup.bash 는 다른 워크스페이스와 패키지가 겹칠 수 있어
# 컨테이너 기본 .bashrc 에서 source 하지 않는다(ros2_ws/xarm_ws/camera_ws만
# 자동 source). 이 스크립트는 서브쉘 안에서만 slam_ws를 source 하므로
# 스크립트를 실행한 쉘이나 다른 프로세스의 환경에는 영향을 주지 않는다.
# -----------------------------------------------------------------------------

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
SLAM_WS="$PROJECT_ROOT/ros2/slam_ws"

TARGET_DOMAIN="${ROS_DOMAIN_ID:-}"
SLAM_DOMAIN=35
IP_ADDRESS="192.168.11.1"

usage() {
    echo "Usage: $0 [-t target_domain] [-s slam_domain] [-i ip_address]"
    echo "  -t <domain>   메인(타겟) 도메인. 미지정 시 현재 ROS_DOMAIN_ID 사용"
    echo "  -s <domain>   Slamware 전용 도메인 (기본값: ${SLAM_DOMAIN})"
    echo "  -i <ip>       Slamware 로봇 IP (기본값: ${IP_ADDRESS})"
    exit 1
}

while getopts ":t:s:i:h" opt; do
    case "$opt" in
        t) TARGET_DOMAIN="$OPTARG" ;;
        s) SLAM_DOMAIN="$OPTARG" ;;
        i) IP_ADDRESS="$OPTARG" ;;
        h) usage ;;
        \?) echo "❌ Unknown option: -$OPTARG"; usage ;;
        :) echo "❌ Option -$OPTARG requires an argument"; usage ;;
    esac
done

if [ -z "$TARGET_DOMAIN" ]; then
    echo "❌ ROS_DOMAIN_ID가 설정되어 있지 않습니다. 컨테이너 환경을 확인하거나 -t로 지정하세요."
    exit 1
fi

if [ "$TARGET_DOMAIN" = "$SLAM_DOMAIN" ]; then
    echo "❌ target_domain(${TARGET_DOMAIN})과 slam_domain(${SLAM_DOMAIN})이 같습니다. 분리된 값을 사용하세요."
    exit 1
fi

if [ ! -f "$SLAM_WS/install/setup.bash" ]; then
    echo "❌ $SLAM_WS/install/setup.bash 가 없습니다. 먼저 slam_ws를 빌드하세요. (alias: slam_build)"
    exit 1
fi

echo "============================================================"
echo "🚀 Mobile(Slamware) Bringup"
echo "============================================================"
echo "Slam domain   : $SLAM_DOMAIN  (slamware_ros_sdk_server_node)"
echo "Target domain : $TARGET_DOMAIN  (slamtec_bridge_node <-> 나머지 시스템)"
echo "IP address    : $IP_ADDRESS"
echo "============================================================"

PIDS=()

cleanup() {
    echo
    echo "🛑 Stopping mobile bringup..."
    for pid in "${PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null
}
trap cleanup INT TERM

# -----------------------------------------------------------------------------
# 1. Slamware 드라이버 (SLAM_DOMAIN 전용)
# -----------------------------------------------------------------------------
(
    source "$SLAM_WS/install/setup.bash"
    export ROS_DOMAIN_ID="$SLAM_DOMAIN"
    exec ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:="$IP_ADDRESS"
) &
PIDS+=($!)

sleep 2  # 드라이버가 먼저 뜬 뒤 브릿지가 붙도록 약간 대기

# -----------------------------------------------------------------------------
# 2. SLAM_DOMAIN <-> TARGET_DOMAIN 브릿지
#    (slamware_ros_sdk 메시지 import를 위해 slam_ws 오버레이 필요)
# -----------------------------------------------------------------------------
(
    source "$SLAM_WS/install/setup.bash"
    export ROS_DOMAIN_ID="$TARGET_DOMAIN"
    exec ros2 run kaair_mobile_bridge slamtec_bridge_node \
        --main-domain "$TARGET_DOMAIN" --slam-domain "$SLAM_DOMAIN"
) &
PIDS+=($!)

wait
