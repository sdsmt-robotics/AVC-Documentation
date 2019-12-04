# Table of Contents
- [Table of Contents](#table-of-contents)
- [Single Board Computer](#single-board-computer)
  - [Odroid](#odroid)
    - [Set up System](#set-up-system)
  - [Raspberry Pi](#raspberry-pi)
    - [Set up System](#set-up-system-1)
  - [ROS](#ros)


# Single Board Computer
A single board computer (SBC) is a system that does not need extra components plugged into it besides a storage device with a OS flashed to it. All of the input and output (I/O), central processing unit (cpu), memory, and other components that are required for the computer to run built into it. The most popular SBC is the Raspberry Pi. Others exist that are just as powerful or more so like the Odroid and Nvidia Jetson.

## Odroid
Hardkernel is the company that develops Odroids. THey make the Odroid-XU4 and Odroid-C2. The C2 is the direct competitor to the Raspberry Pi in 2016. Both of these systems are ARM based so they require different setup instructions then a computer running an Intel or AMD cpu. The Odroid-N2 is the most rescent ARM SBC the company has created. They have an x86_64 SBC that uses a celeron.

### Set up System
It is recommended to follow the official documentaion for flashing the Odroid. I would suggest using the minimum image installations since these will most likely be running headless on robot.

https://wiki.odroid.com/getting_started/os_installation_guide#operating_systems_we_re_providing

## Raspberry Pi
The Raspberry Pi is the most widely known SBC at the moment. Just recently it realesed version 4B that has up to 4GB of RAM and Gigabit ethernet. 

### Set up System
The raspberry pi will be similar to the odroid in how the OS gets written to the SD card. Once the following instuctions are followed, it is recommended to change user password if the default is still used.

https://ubuntu.com/download/raspberry-pi

## ROS
The instructions for installing on the Odroid and Raspberry Pi are essentially the same:
http://wiki.ros.org/Installation/UbuntuARM

We probably don't want to do a desktop full installation on the SBC. We would only want desktop at the most with other packages that are required being individually installed when doing testing and debugging. If everything is working well we can install just the base ROS installation along with specifically needed ROS packages when deploying the robot.
