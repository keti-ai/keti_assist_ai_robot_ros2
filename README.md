# KETI Assist AI Robot ROS 2 Packages

This repository is an updated version of the kcare_robot_ros2_controller ROS 2 package.
It improves upon the original repository to enable easier and more seamless use of ROS 2 packages such as controllers, descriptions, and MoveIt. This package offers core interfaces for the control of robot vision, pan-tilt mechanisms, lift systems, and manipulators, serving as an integrated solution for physical AI applications. For detailed usage instructions, please refer to the documentation below.

- [Documentation for KETI Assist AI ROBOT](https://bittersweet-singer-2da.notion.site/2fcb18ffad91806cbfd2c2f29d54d1cd?v=2fcb18ffad9180a6b2e6000c43cecc10&source=copy_link)


## How To Use Quickly

- ### Obtain source code of "keti_assist_ai_robot_ros2" repository
  ```bash
  # Just Clone this repository. Repository has its own ros2 workspace directory
  git clone https://github.com/keti-ai/keti_assist_ai_robot_ros2.git
  ```

- ### Import third-party packages with vcstool
  External ROS 2 packages are listed in distro-specific repos files and imported into separate workspaces:
  - `xarm_ros2` → `ros2/xarm_ws` (`humble` or `jazzy` branch)
  - `OrbbecSDK_ROS2`, `realsense-ros` → `ros2/camera_ws`
  - KAAIR packages stay in `ros2/ros2_ws`
  Docker 이미지에는 이 패키지들을 넣지 않는다. 컨테이너 기동 후 아래처럼 가져온 뒤 각 워크스페이스에서 빌드한다.
  ```bash
  cd keti_assist_ai_robot_ros2
  sudo apt update && sudo apt install -y vcstool
  bash scripts/import_third_party.sh -t humble   # or: -t jazzy
  ```
  You can also import directly from the repository root:
  ```bash
  vcs import --recursive . < third_party.humble.repos
  # vcs import --recursive . < third_party.jazzy.repos
  ```
  Update already imported repositories with:
  ```bash
  vcs pull ros2/xarm_ws ros2/camera_ws
  ```

- ### Setup Network Buffer
  #### Resize Network socket buffer size for ROS2 DDS Communication optimization
  ```bash
  cd keti_assist_ai_robot_ros2
  bash scripts/setup_network_buffer.sh
  ```

- ### Build with Docker
  #### Move to Repository directory
    ```bash
    cd keti_assist_ai_robot_ros2
    ```
  #### Build Docker Image .sh what you want
  ```bash
  bash scripts/build_docker.sh
  ```
  #### Move to Workspace Directory and docker-compose up. You can edit docker-compse.yml and rerun this code.
  ```bash
  docker compose up -f docker-compose.cpu.yml up -d
  docker compose up -f docker-compose.jetpack.yml up -d
  # If you want to change ROS distro, edit .env or input args
  ROS_DISTRO=humble docker compose -f docker-compose.cpu.yml up -d
  ```
  #### Attach to docker shell
  ```bash
  docker exec -it keti_ros2_container /bin/bash
  ```

  #### In the docker shell, import third-party sources then build each workspace
  ```bash
  bash scripts/import_third_party.sh -t humble   # or: -t jazzy
  cd /ros_ws/ros2/xarm_ws && colcon build --symlink-install
  cd /ros_ws/ros2/camera_ws && colcon build --symlink-install
  cd /ros_ws/ros2/ros2_ws && colcon build --symlink-install
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /ros_ws/ros2/xarm_ws/install/setup.bash
  source /ros_ws/ros2/camera_ws/install/setup.bash
  source /ros_ws/ros2/ros2_ws/install/setup.bash
  ```

  #### Test Kaair Fake MoveIT is working
  ```bash
  ros2 launch kaair_moveit_config kaair_moveit.launch.py use_fake_hardware:=true
  ```
