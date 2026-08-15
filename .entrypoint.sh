#!/bin/bash
# =====================================================
# 공통 셸 환경 설정 (root / 일반 사용자 공통)
# .bashrc 에서 source /workspace/.entrypoint.sh 로 로드됨
# =====================================================

export TERM=xterm-256color

# ── Aliases ──────────────────────────────────────────
alias eb="nano ~/.bashrc"
alias sb="source ~/.bashrc"
alias cbs="cd /workspace/ros2/ros2_ws && colcon build --symlink-install"
alias xarm_build="cd /workspace/ros2/xarm_ws && colcon build --symlink-install --packages-up-to xarm_api xarm_controller uf_ros_lib"
alias slam_build="cd /workspace/ros2/slam_ws && colcon build --symlink-install"
alias ob_build="cd /workspace/ros2/camera_ws && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"
alias cclean="rm -rf /workspace/ros2/ros2_ws/build /workspace/ros2/ros2_ws/install /workspace/ros2/ros2_ws/log"

# ── Functions ─────────────────────────────────────────
cclean_all() {
    for ws in /workspace/ros2/xarm_ws /workspace/ros2/camera_ws /workspace/ros2/slam_ws /workspace/ros2/ros2_ws; do
        rm -rf "$ws/build" "$ws/install" "$ws/log"
    done
}

# ── ROS 2 환경 소싱 ───────────────────────────────────
source /opt/ros/$ROS_DISTRO/setup.bash
for ws in /workspace/ros2/xarm_ws /workspace/ros2/camera_ws /workspace/ros2/ros2_ws; do
    [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash"
done

# ── 사용자별 PS1 구분 (root = 빨강, 일반 사용자 = 청록) ──
if [ "$(id -u)" = "0" ]; then
    export PS1="\[\e[01;31m\]\u@container\[\e[00m\]:\[\e[01;37m\]\w\[\e[00m\]# "
else
    export PS1="\[\e[01;36m\]\u@docker\[\e[00m\]:\[\e[01;33m\]\w\[\e[00m\]\$ "
fi
