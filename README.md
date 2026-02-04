# KETI Assist AI Robot ROS 2 Packages

This repository is an updated version of the kcare_robot_ros2_controller ROS 2 package.
It improves upon the original repository to enable easier and more seamless use of ROS 2 packages such as controllers, descriptions, and MoveIt. This package offers core interfaces for the control of robot vision, pan-tilt mechanisms, lift systems, and manipulators, serving as an integrated solution for physical AI applications. For detailed usage instructions, please refer to the documentation below.

- [Documentation for KETI Assist AI ROBOT](https://bittersweet-singer-2da.notion.site/2fcb18ffad91806cbfd2c2f29d54d1cd?v=2fcb18ffad9180a6b2e6000c43cecc10&source=copy_link)


## How To Use Quickly

- ### Obtain source code of "keti_assist_ai_robot_ros2" repository
  ```bash
  # Just Clone this repository. Repository has its own ros2 workspace directory
  git clone https://github.com/keti-ai/keti_assist_ai_robot_ros2.git

  # Move to repository
  cd keti_assist_ai_robot_ros2

  # get third party submodules
  git submodule update --remote --recursive
  ```

- ### Build ROS2 Package
  #### Move to Repository directory
  ```bash
  # Remember to source ros2 environment settings first
  cd <repository_directory>
  ```

  #### Build all packages (Release Mode, except xarm_sdk(Debug build need))
  ```bash
  colcon build --symlink-install \
  --metas .build_config/colcon_build.meta \
  --packages-up-to $(cat .build_config/pkgs-control.txt) \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
  - #### Tip for Python venv/conda users
    If the `colcon build` command is not directly recognized or you need to ensure the specific interpreter is used, prefix the command with `python -m`:

    `python -m colcon build --symlink-install --metas .build_config/colcon_build.meta --packages-up-to $(cat .build_config/pkgs-control.txt) --cmake-args -DCMAKE_BUILD_TYPE=Release`

  #### If you want to build interface only
  ```bash
  colcon build --symlink-install \
  --packages-up-to $(cat .build_config/pkgs-interfaces.txt) \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

  - #### For Python venv/conda users
    `python -m colcon build --symlink-install --packages-up-to $(cat .build_config/pkgs-interfaces.txt) --cmake-args -DCMAKE_BUILD_TYPE=Release`