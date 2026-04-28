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
  # change input argument (cpu,nvidia,jetpack)
  bash scripts/build_docker.sh -t nvidia
  ```
  #### This copies `docker-compose.yml` in your workspace repository
  #### Move to Workspace Directory and docker-compose up. You can edit docker-compse.yml and rerun this code.
  ```bash
  docker-compose up -d
  ```
  #### Attach to docker shell
  ```bash
  docker exec -it keti_ros2_container /bin/bash
  ```

  #### In the docker shell, build ros2 packages using colcon command
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

  #### Test Kaair Fake MoveIT is working
  ```bash
  ros2 launch kaair_moveit_config kaair_moveit.launch.py use_fake_hardware:=true
  ```
