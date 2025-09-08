# bldc_actuator_ws

This repository contains a development environment and interface integration of Odrive3.6 ROS2 package with a BLDC robotic actuator.

## Open de workspace

1. [Install Docker and VSCode.](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
2. `sudo docker image pull ros:humble`
3. Use `View->Command Palette...` or `Ctrl+Shift+P` to open the command palette. Search for the command `Dev Containers: Reopen in Container` and execute it. This will build your development docker container for your.

## To build devcontainer image for fully automated start

1. `sudo apt install npm`
2. Install Dev Container CLI (need sudo priviliege): `sudo npm install -g @devcontainers/cli`
3. Start the devcontainer: `devcontainer up --workspace-folder .`
4. You wan't to interact with a container's terminal you can run: `devcontainer exec --workspace-folder <folder> /bin/sh`

## From ws ROS 2 folder

``` bash
source /opt/ros/$DISTRO/setup.bash
colcon build
source install/local_setup.bash
```

## Configuring CAN interface

With Raspberry pi there is no CAN interface by default, but SPI can be used together with a CAN controller, in my case I'm using bcm2835 chip in a 2Ch HAT format. This HAT(<https://www.waveshare.com/wiki/2-CH_CAN_HAT+>) has isolated ground for avoiding gnd loops and protect RPI from induced currents from BLDC motors.

The configuration will depends on the CAN controller you have:

``` bash

wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.60.tar.gz
tar zxvf bcm2835-1.60.tar.gz 
cd bcm2835-1.60/
sudo ./configure
sudo make
sudo make check
sudo make install

sudo dmesg | grep spi1

sudo ip link set can0 down
sudo ip link set can1 down

sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can1 txqueuelen 65536

ifconfig

ip -details -statistics link show can0
ip -details -statistics link show can1

candump can0
candump can1

candump -tz can0
```

## Setup actuator

''' bash
You can test and configure actuator at test/actuators/ directory.

dump_errors(odrv0)

odrv0.clear_errors()

odrv0.axis0.motor.motor_thermistor

'''

### Configure actuator script:

`configure_actuator.py`

1. Load nominal actuator values and configuration.

``` bash
# save actual configuration
odrivetool backup-config path/to/file/test.json

# load a configuration file 
odrivetool restore-config path/to/file/test.json
```

2. Start calibration sequence to set up phase_resistance and phase_inductance specific values.

Usage of the script:



``` bash
cd tests/actuators/

## make sure output axis is free to rotate, secuence do more than one turn
python3 configure_actuator.py --node_id 2 --r120 False

usage: configure_actuator.py [-h] [--node_id NODE_ID] [--r120 R120] [--config CONFIG]

Configura un actuador ODrive con parámetros desde JSON

options:
  -h, --help         show this help message and exit
  --node_id NODE_ID  Node ID para CAN (por defecto: 0)
  --r120 R120        Enable built in terminate resistor: True or False
  --config CONFIG    Archivo JSON de configuración

```

### Set actuator index

Zero actual posion of the actuator. Execute following script:

`calibrate_actuator.py`

``` bash

python3 calibrate_actuator.py

TODO: incluir argumento que defina la posición deseada del actuador.
```

### Test_actuator

Onces all verifications are done and configuration is correct, run `test_actuator.py`. It performs a slow homing to zero position and a secuence of sweep movements, **keep motor shaft free for this trial**.  
