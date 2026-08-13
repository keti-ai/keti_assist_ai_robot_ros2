# ==========================================
# 1. 베이스 이미지 및 환경 설정
# ==========================================
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG TARGETARCH

# ROS의 setup.bash는 ${BASH_SOURCE[0]} 등 bash 전용 문법을 사용하므로,
# 기본 셸(/bin/sh -> dash)이 아닌 bash로 RUN 명령을 실행하도록 지정한다.
SHELL ["/bin/bash", "-c"]


# ==========================================
# 2. 시스템 기본 도구 및 ROS 2 리포지토리 설정
# ==========================================
RUN apt-get update && apt-get install -y \
    locales nano net-tools git curl wget gnupg2 lsb-release ca-certificates \
    build-essential cmake gdb software-properties-common \
    python3-pip python-is-python3 \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 2-1. pip 전역 설정 (PEP 668 대응)
# ---------------------------------------------------------
# Ubuntu 24.04(Noble, jazzy)부터는 시스템 pip가 PEP 668(externally-managed-environment)로
# 보호되어 기본적으로 설치가 차단된다. 매번 --break-system-packages를 넘기지 않도록
# 해당 환경일 때만 전역 pip 설정(/etc/pip.conf)에 반영해, 이후 이 Dockerfile 내
# RUN pip install은 물론 컨테이너 내부에서 root/일반 사용자가 직접 실행하는
# pip install에도 동일하게 적용되게 한다.
# (humble 등 Ubuntu 22.04의 구버전 pip는 이 마커 파일 자체가 없으므로 그대로 둔다)
RUN if ls /usr/lib/python3*/EXTERNALLY-MANAGED >/dev/null 2>&1; then \
        printf '[global]\nbreak-system-packages = true\n' > /etc/pip.conf; \
    fi


# ==========================================
# 3. ROS 2 기본 데스크탑 및 개발 도구
# ==========================================
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-1. ROS 2 Control 핵심 (Control, Manager, HW IF)
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-2. 표준 컨트롤러 및 브로드캐스터
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-3. MoveIt 2 관련 플러그인
# ---------------------------------------------------------
    RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-moveit-ros-planning \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-moveit-ros-perception \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-octomap-ros \
    ros-$ROS_DISTRO-moveit-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-4. 이미지 및 비전 관련 플러그인
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libgflags-dev nlohmann-json3-dev \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
    ros-$ROS_DISTRO-backward-ros libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev \
    ros-$ROS_DISTRO-image-view ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-find-object-2d \
    ros-$ROS_DISTRO-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-5. Navigation 2 관련 플러그인
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-nav2-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-6. 하드웨어 SDK (Dynamixel, Modbus, RealSense Binary)
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-dynamixel-sdk \
    libmodbus-dev \
    ros-$ROS_DISTRO-librealsense2* \
    && rm -rf /var/lib/apt/lists/*

# ==========================================
# 4. RViz2 버전 설정
# ==========================================
# - Humble: 아키텍처별 스냅샷 버전 고정 설치
# - Jazzy 이상: 별도 고정 없이 배포판 최신 rviz2 사용
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
        apt-get update && apt-get install -y --no-install-recommends wget ca-certificates \
        && ARCH=$(dpkg --print-architecture) \
        && case "$ARCH" in \
            amd64) URL="http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250617.234657_amd64.deb" ;; \
            arm64) URL="http://snapshots.ros.org/humble/2025-06-18/ubuntu/pool/main/r/ros-humble-rviz-common/ros-humble-rviz-common_11.2.17-1jammy.20250607.163324_arm64.deb" ;; \
            *) echo "❌ Unsupported architecture for rviz pinning: $ARCH" && exit 1 ;; \
        esac \
        && echo "Downloading rviz-common for $ARCH..." \
        && wget "$URL" -O /tmp/rviz-common.deb \
        && dpkg -i /tmp/rviz-common.deb || apt-get install -y -f \
        && apt-mark hold ros-humble-rviz-common \
        && rm /tmp/rviz-common.deb \
        && rm -rf /var/lib/apt/lists/* ; \
    else \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-$ROS_DISTRO-rviz2 \
        && rm -rf /var/lib/apt/lists/* ; \
    fi


# xArm / 카메라 ROS 패키지는 이미지에 넣지 않는다.
# 컨테이너 기동 후 scripts/import_third_party.sh 로 가져온 뒤
# ros2/xarm_ws, ros2/camera_ws 에서 colcon build 한다.

# ==========================================
# 8. Cyclone DDS Installation
# ==========================================
RUN apt-get update && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp


ARG USERNAME=admin
ARG USER_UID=1000
ARG USER_GID=1000

# sudo 패키지 설치 및 사용자 권한 부여
RUN apt-get update && apt-get install -y sudo \
    && if id -u ${USER_UID} >/dev/null 2>&1; then \
            EXIST_USER=$(getent passwd ${USER_UID} | cut -d: -f1); \
            usermod -l ${USERNAME} ${EXIST_USER}; \
            usermod -d /home/${USERNAME} -m ${USERNAME}; \
        else \
            groupadd --gid ${USER_GID} ${USERNAME}; \
            useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
        fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}


# 환경 변수 및 Alias 설정 (root 및 사용자 공통)
RUN USER_HOME=$(getent passwd $(cat /tmp/container_user) | cut -d: -f6) && \
    for user_home in /root ${USER_HOME}; do \
        echo 'export TERM=xterm-256color' >> ${user_home}/.bashrc && \
        echo 'alias eb="nano ~/.bashrc"' >> ${user_home}/.bashrc && \
        echo 'alias sb="source ~/.bashrc"' >> ${user_home}/.bashrc && \
        echo 'alias cbs="cd /workspace/ros2/ros2_ws && colcon build --symlink-install"' >> ${user_home}/.bashrc && \
        echo 'alias xarm_build="cd /workspace/ros2/xarm_ws && colcon build --symlink-install --packages-up-to xarm_api xarm_controller uf_ros_lib"' >> ${user_home}/.bashrc && \
        echo 'alias slam_build="cd /workspace/ros2/slam_ws && colcon build --symlink-install"' >> ${user_home}/.bashrc && \
        echo 'alias ob_build="cd /workspace/ros2/camera_ws && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"' >> ${user_home}/.bashrc && \
        echo 'alias cclean="rm -rf build/ install/ log/"' >> ${user_home}/.bashrc; \
        echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ${user_home}/.bashrc; \
        echo 'for ws in /workspace/ros2/xarm_ws /workspace/ros2/camera_ws /workspace/ros2/ros2_ws; do [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash"; done' >> ${user_home}/.bashrc; \
    done

# 사용자별 터미널 색상 구분
RUN echo 'export PS1="\[\e[01;36m\]\u@docker\[\e[00m\]:\[\e[01;33m\]\w\[\e[00m\]\$ "' >> /home/${USERNAME}/.bashrc && \
    echo 'export PS1="\[\e[01;31m\]\u@container\[\e[00m\]:\[\e[01;37m\]\w\[\e[00m\]# "' >> /root/.bashrc

# ==========================================
# 11. 최종 작업 환경 설정
# ==========================================
WORKDIR /workspace
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt


ENV SHELL=/bin/bash

USER ${USERNAME}
CMD ["/bin/bash"]
