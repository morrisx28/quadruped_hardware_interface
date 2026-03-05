# Quadruped Hardware Interface
DDS hardware interface for CSL quadruped robot
## Overview
### Hardware
- Joint motors: Damiao DM4310
- IMU: Xsens Mti320

## Build
1. Follow this repo to install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).
2. Set up IMU lib
    ```
    cd ~/pineapple_hardware_interface/src/xspublic
    make clean
    make
    ```
3. Set up motor lib
    ```
    sudo apt install libusb-1.0-0
    sudo nano /etc/udev/rules.d/99-usb.rules
    ```
    add following line
    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    ```
    cd ~/quadruped_hardware_interface
    mkdir build
    cd build
    cmake ..
    make
    ```
## Useage

```
cd ~/quadruped_hardware_interface/build
sudo ./quadruped_hardware_interface
```
