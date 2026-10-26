# ==========================================
# 1. 베이스 이미지 및 환경 설정
# ==========================================
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO

# ROS의 setup.bash는 ${BASH_SOURCE[0]} 등 bash 전용 문법을 사용하므로,
# 기본 셸(/bin/sh -> dash)이 아닌 bash로 RUN 명령을 실행하도록 지정한다.
SHELL ["/bin/bash", "-c"]


# ---------------------------------------------------------
# 1-1. apt-get/apt 재시도 래퍼 설치
# ---------------------------------------------------------
# apt-get install 중 "Hash Sum mismatch"가 간헐적으로 발생한다(원인은 특정
# 패키지가 아니라, 한 번에 받는 다운로드 용량이 크고 연결이 길게 유지될수록
# 전송 도중 데이터가 손상될 확률이 올라가는 것으로 보임 — 매번 다른 패키지가,
# 크기는 같고 해시만 다르게 깨져서 들어온다). ros-desktop 처럼 패키지 하나가
# 수백 개의 의존성을 한 번에 끌어오는 단계는 설치 목록을 쪼개서 나눌 수가 없으므로,
# apt-get/apt 호출 자체를 가로채 실패 시 최대 5회까지 자동으로 재시도하는 래퍼를
# 깐다. dpkg/apt는 이미 설치된 패키지는 건너뛰므로, 재시도할 때마다 손상됐던
# 나머지 패키지만 다시 받다가 결국 전체가 성공한다. /usr/local/bin 이 PATH 상
# /usr/bin 보다 먼저 탐색되므로, 이 아래의 모든 apt-get/apt 호출에 별도 수정
# 없이 자동으로 적용된다. (외부 파일 의존 없이 Dockerfile 안에서만 해결하기
# 위해 COPY 대신 printf로 스크립트를 직접 생성한다.)
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        'name=$(basename "$0")' \
        'real="/usr/bin/$name"' \
        'cmd="$1"' \
        'if [ "$cmd" = "update" ] || [ "$cmd" = "install" ]; then' \
        '    n=0' \
        '    until "$real" "$@"; do' \
        '        n=$((n + 1))' \
        '        if [ "$n" -ge 5 ]; then' \
        '            exec "$real" "$@"' \
        '        fi' \
        '        echo "[apt-retry] $name $cmd failed (attempt $n/5), retrying..." >&2' \
        '        sleep 3' \
        '    done' \
        'else' \
        '    exec "$real" "$@"' \
        'fi' \
        > /usr/local/bin/apt-get-retry.sh \
    && chmod +x /usr/local/bin/apt-get-retry.sh \
    && cp /usr/local/bin/apt-get-retry.sh /usr/local/bin/apt-get \
    && cp /usr/local/bin/apt-get-retry.sh /usr/local/bin/apt \
    && chmod +x /usr/local/bin/apt-get /usr/local/bin/apt


# ==========================================
# 2. 시스템 기본 도구 및 ROS 2 리포지토리 설정
# ==========================================
# 위 1-1의 재시도 래퍼가 ros-desktop 처럼 큰 설치 단계도 통째로 재시도해주므로,
# 이 단계를 잘게 쪼개서 재시도 단위를 줄여야 할 이유는 더 이상 없다. 하나의
# RUN으로 합쳐 apt-get update 메타데이터(~40MB)를 여러 번 받지 않게 하고,
# 같은 레이어 안에서 rm -rf /var/lib/apt/lists/* 로 정리해 이미지 용량도 줄인다.
RUN apt-get update && apt-get install -y \
    locales nano net-tools git curl wget gnupg2 lsb-release ca-certificates \
    build-essential cmake gdb software-properties-common \
    python3-pip python-is-python3 \
    && locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8 \
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
RUN (apt-get update || true) && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-1. ROS 2 Control 핵심 (Control, Manager, HW IF)
# ---------------------------------------------------------
RUN (apt-get update || true) && apt-get install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-2. 표준 컨트롤러 및 브로드캐스터
# ---------------------------------------------------------
RUN (apt-get update || true) && apt-get install -y \
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
RUN (apt-get update || true) && apt-get install -y \
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
RUN (apt-get update || true) && apt-get install -y \
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
RUN (apt-get update || true) && apt-get install -y \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-nav2-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-5-1. RViz2 2D Overlay 플러그인 (servo 상태 등 화면 고정 텍스트 HUD)
# ---------------------------------------------------------
# Jazzy(noble)의 ros-jazzy-rviz-2d-overlay-plugins 바이너리(.deb)는 apt 미러의
# 모든 스냅샷(과거~현재)에 걸쳐 librviz_2d_overlay_plugins.so 의
# OverlayTextDisplay vtable 심볼(_ZTVN23rviz_2d_overlay_plugins18OverlayTextDisplayE)
# 이 undefined 로 빠져 있어, rviz2 에서 "PluginlibFactory: ... failed to load ...
# undefined symbol" 로 로드가 실패한다(버전 skew 가 아니라 패키징 자체의 결함 —
# 동일 소스를 직접 colcon build 하면 vtable 이 정상적으로 나온다).
# → Jazzy 는 apt 대신 소스 빌드로 대체한다: third_party.jazzy.repos 에
#   ros2/ros2_ws/src/third_party/rviz_2d_overlay_plugins 로 vcs import 되고,
#   ros2_ws 를 colcon build 할 때 함께 빌드된다(scripts/import_third_party.sh 참고).
# Humble 의 apt 패키지는 이 결함이 없으므로 그대로 둔다.
RUN (apt-get update || true) && apt-get install -y \
    ros-$ROS_DISTRO-rviz-2d-overlay-msgs \
    && if [ "$ROS_DISTRO" = "humble" ]; then \
        apt-get install -y ros-$ROS_DISTRO-rviz-2d-overlay-plugins ; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# 3-6. 하드웨어 SDK (Dynamixel, Modbus, RealSense)
# ---------------------------------------------------------

RUN (apt-get update || true) && apt-get install -y \
    ros-$ROS_DISTRO-dynamixel-sdk \
    libmodbus-dev \
    && rm -rf /var/lib/apt/lists/*

# RealSense 공식 APT 저장소 등록
RUN set -eux; \
    mkdir -p /etc/apt/keyrings; \
    curl -fsSL \
      https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
      -o /tmp/librealsenseai.asc; \
    gpg --batch --dearmor \
      -o /etc/apt/keyrings/librealsenseai.gpg \
      /tmp/librealsenseai.asc; \
    chmod 644 /etc/apt/keyrings/librealsenseai.gpg; \
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/librealsense.list; \
    rm /tmp/librealsenseai.asc

# RealSense SDK 설치
RUN (apt-get update || true) && \
    apt-get install -y --no-install-recommends \
      librealsense2-dev \
      librealsense2-utils \
      librealsense2-udev-rules \
    && rm -rf /var/lib/apt/lists/*

# ==========================================
# 4. RViz2 버전 설정
# ==========================================
# - Humble: 아키텍처별 스냅샷 버전 고정 설치
# - Jazzy 이상: 별도 고정 없이 배포판 최신 rviz2 사용
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
        (apt-get update || true) && apt-get install -y --no-install-recommends wget ca-certificates \
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
        (apt-get update || true) && apt-get install -y --no-install-recommends \
            ros-$ROS_DISTRO-rviz2 \
        && rm -rf /var/lib/apt/lists/* ; \
    fi


# xArm / 카메라 ROS 패키지는 이미지에 넣지 않는다.
# 컨테이너 기동 후 scripts/import_third_party.sh 로 가져온 뒤
# ros2/xarm_ws, ros2/camera_ws 에서 colcon build 한다.

# ==========================================
# 8. Cyclone DDS Installation
# ==========================================
RUN (apt-get update || true) && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*


ARG USERNAME=admin
ARG USER_UID=1000
ARG USER_GID=1000

# sudo 패키지 설치 및 사용자 권한 부여
RUN (apt-get update || true) && apt-get install -y sudo \
    && if id -u ${USER_UID} >/dev/null 2>&1; then \
            EXIST_USER=$(getent passwd ${USER_UID} | cut -d: -f1); \
            usermod -l ${USERNAME} ${EXIST_USER}; \
            usermod -d /home/${USERNAME} -m ${USERNAME}; \
        else \
            groupadd --gid ${USER_GID} ${USERNAME}; \
            useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
        fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*


# 환경 변수·Alias·PS1 설정은 /workspace/.entrypoint.sh 에 모아두고,
# root / 일반 사용자 양쪽 .bashrc 에서 source 한 줄로 로드한다.
RUN for user_home in /root /home/${USERNAME}; do \
        echo 'source /workspace/.entrypoint.sh' >> ${user_home}/.bashrc; \
    done

# ==========================================
# 11. 최종 작업 환경 설정
# ==========================================
WORKDIR /workspace
COPY requirements.txt .
RUN pip install --no-cache-dir --ignore-installed -r requirements.txt


ENV SHELL=/bin/bash

USER ${USERNAME}
CMD ["/bin/bash"]
