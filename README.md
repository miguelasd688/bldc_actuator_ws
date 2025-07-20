# bldc_actuator_ws
This repository contains a development environment and interface integration of Odrive3.6 ROS2 package with a BLDC robotic actuator.


# Open de workspace

1. [Install Docker and VSCode.](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
2. Use `View->Command Palette...` or `Ctrl+Shift+P` to open the command palette. Search for the command `Dev Containers: Reopen in Container` and execute it. This will build your development docker container for your. 

# To build devcontainer image for fully automated start
1. `sudo apt install npm`
2. Install Dev Container CLI (need sudo priviliege): `sudo npm install -g @devcontainers/cli`
3. Start the devcontainer: `devcontainer up --workspace-folder .` 
4. You wan't to interact with a container's terminal you can run: `devcontainer exec --workspace-folder <folder> /bin/sh`


# From ws ROS 2 folder:

```
source /opt/ros/$DISTRO/setup.bash
colcon build
source install/local_setup.bash
```

