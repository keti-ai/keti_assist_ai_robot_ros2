#!/usr/bin/env bash
#
# ros2_dds_clean.sh
# 호스트 OS에서 ROS2 DDS 통신 이상(discovery 실패, 간헐적 끊김 등)이 발생했을 때
# 재부팅 없이 관련 상태를 정리하는 스크립트.
#
# 사용법:
#   sudo ./ros2_dds_clean.sh          # 기본: 프로세스 정리 + shm 정리 + daemon 재시작
#   sudo ./ros2_dds_clean.sh --net    # 위 내용 + 네트워크 인터페이스 재시작까지 포함
#
# 주의: 이 스크립트는 호스트에서 돌고 있는 ROS2 프로세스를 강제 종료합니다.
#       실행 전에 돌아가고 있는 노드/런치가 있으면 먼저 안전 종료를 시도하세요.

set -uo pipefail

NET_RESET=0
if [[ "${1:-}" == "--net" ]]; then
  NET_RESET=1
fi

echo "=== [1/5] 실행 중인 ROS2 관련 프로세스 확인 ==="
pgrep -af 'ros2|_ros2_daemon|rclpy|rclcpp' || echo "  (관련 프로세스 없음)"

echo
echo "=== [2/5] ROS2 daemon 정지 ==="
ros2 daemon stop 2>/dev/null || echo "  (daemon 미실행 또는 ros2 CLI 없음)"

echo
echo "=== [3/5] 남아있는 ROS2/DDS 프로세스 강제 종료 ==="
# ros2 daemon, 그리고 비정상 종료 후 남은 rclpy/rclcpp 프로세스까지 정리
pkill -9 -f '_ros2_daemon' 2>/dev/null
pkill -9 -f 'ros2 daemon'  2>/dev/null
# 아래 줄은 "호스트에서 직접 실행 중인" ROS2 노드까지 모두 죽입니다.
# 컨테이너 안 프로세스는 대상이 아닙니다(도커는 별도 namespace).
pkill -9 -f 'rclpy'  2>/dev/null
pkill -9 -f 'rclcpp' 2>/dev/null

echo
echo "=== [4/5] 잔여 shared memory / DDS 임시 파일 정리 ==="
# Fast DDS(기본 RMW) shared-memory 세그먼트
if ls /dev/shm/fastrtps_* >/dev/null 2>&1; then
  rm -rf /dev/shm/fastrtps_*
  echo "  /dev/shm/fastrtps_* 삭제 완료"
else
  echo "  (fastrtps shm 세그먼트 없음)"
fi
# Cyclone DDS를 쓰는 경우 대비
if ls /dev/shm/cyclonedds_* >/dev/null 2>&1; then
  rm -rf /dev/shm/cyclonedds_*
  echo "  /dev/shm/cyclonedds_* 삭제 완료"
fi
# 일부 환경에서 SHM 세그먼트가 System V ipc로 잡히는 경우 확인만 (자동 삭제는 위험해서 안 함)
if command -v ipcs >/dev/null 2>&1; then
  echo "  --- 참고: 현재 System V shared memory 세그먼트 ---"
  ipcs -m | tail -n +4
fi

if [[ "$NET_RESET" -eq 1 ]]; then
  echo
  echo "=== [5/5] 네트워크 인터페이스 재시작 (--net 옵션) ==="
  # 기본 이더넷/와이파이 인터페이스 이름은 환경마다 다르므로 자동 탐지 시도
  IFACE=$(ip -o link show | awk -F': ' '{print $2}' | grep -Ev '^(lo|docker|br-|veth)' | head -n1)
  if [[ -n "$IFACE" ]]; then
    echo "  대상 인터페이스: $IFACE"
    ip link set "$IFACE" down
    sleep 1
    ip link set "$IFACE" up
    echo "  $IFACE down/up 완료"
  else
    echo "  자동 탐지 실패 — 수동으로 'sudo ip link set <iface> down/up' 실행 필요"
  fi
else
  echo
  echo "=== [5/5] 네트워크 재시작 생략 (필요시 --net 옵션으로 실행) ==="
fi

echo
echo "=== 완료. ros2 daemon은 다음 ros2 CLI 명령 실행 시 자동으로 재기동됩니다. ==="
