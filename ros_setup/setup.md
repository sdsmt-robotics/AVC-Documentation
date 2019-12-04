# Table of Contents
- [Table of Contents](#table-of-contents)
- [Setup Workflow for ROS](#setup-workflow-for-ros)
  - [Installing Linux](#installing-linux)
    - [Dual Boot](#dual-boot)
      - [Before Booting Install Media](#before-booting-install-media)
      - [During Install](#during-install)
    - [Virtual Machine](#virtual-machine)
  - [Native ROS Install](#native-ros-install)
    - [Install ROS](#install-ros)
    - [Catkin Workspace Setup](#catkin-workspace-setup)
  - [Docker ROS](#docker-ros)
    - [Docker Setup](#docker-setup)
    - [Docker and VS Code](#docker-and-vs-code)
    - [Catkin Workspace Setup](#catkin-workspace-setup-1)
- [Developing ROS Packages](#developing-ros-packages)


# Setup Workflow for ROS 
This guide will help you set up a workflow on a desktop or laptop. It won't go into detail on everything and will leave that to the official robot operating system (ROS) documentation. This guide will contain the necessary information required for installation onto Ubuntu and appropriate links. There are other ways to install ROS, however, someone new to ROS can get overwhelmed when something doesn't work right. It is reccomended to fallow the Ubuntu instructions for first time users. Almost all of the resources for fixing issues are on Ubuntu. Fixes here can work for other Linux distributions, but they are not guranteed to work. 

The robot operating system can work on linux in two ways. A native ROS install vs using a container environment such as docker. Another environment is running ROS on Linux inside of a virtual machine. This would require Ubuntu set up in VM Ware, Virtual Box, and others. This is more heavyweight then a container environment, but it would be like a native ROS install for set up. Both methods can work well and they each have their own advantages and disadvantages.  Native ROS installs work the best on Ubuntu and when running applications such as rviz and gazebo are necessary for visulization. Docker works well when running applications without hardware acceleration when all you have is intel graphics. Docker is also great for running on top of other linux distributions that don't have ROS installation instuctions.

## Installing Linux
Linux can be installed in multiple ways. It can be the only operating system on the computer, Multiple drives for different operating system (Each OS gets its own drive), and it can share a boot drive with another such as Windows (dual boot). The reccomended Linux distrubion is Ubuntu. Specifically the long term support release (LTS). As of this writing it is 18.04. ROS officially supports Ubuntu and Debian and that makes it easy to troubleshoot when one comes accross issues. 

Download the image:
https://ubuntu.com/download/desktop

Create a bootable USB stick on Windows and Ubuntu:

https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0

https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#3

Tutorial for installing Ubuntu:
https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0
### Dual Boot
#### Before Booting Install Media
Before you boot off of the USB that Ubuntu was flashed to, you should shrink the C volume in windows in order make room for Linux. It is reccommended to do 50GB at least if you are planning on a desktop full installation of ROS. This version comes with Gazebo (simulator) and many packages for developing. 

It is also a good idea to have quite a bit of space if you are planning on going the docker route. Perhaps more space or having a data drive to store the docker containers will be required.

#### During Install
There is an option when installing Ubuntu to install alongside Windows. This option is displayed at step 6 on the above tutorial. Windows should be auto detected by the installer. It is not displayed in the above tutorial since no OS was detected on any drives.

### Virtual Machine
With a virtual machine you will need to creaete a virtual disk file and that would be the hard drive for Linux. This is done using VM Ware or Virtual Box. It is not necessary to 50GB if you don't want to. However 30-40 should be good for some swap space included. You can mount directories on the host system in the VM so you can have shared space that VM doesn't need to initially create.

User penreturns has good guide on doing this in Virtualbox for installing Ubuntu. https://askubuntu.com/questions/142549/how-to-install-ubuntu-on-virtualbox

## Native ROS Install
The native install of ROS will be required on the odroid. It may not be required on a laptop or desktop. For running applications such as Gazebo and rviz it is recommended. Especially if hardware accellaration is needed for Gazebo to run smoothly. It also should be the method used on single board computer such as the odroid and raspberry pi.

### Install ROS
Follow the instuctions at http://wiki.ros.org/melodic/Installation/Ubuntu to install ROS on ubuntu 18.04. Once ROS is installed the instructions have source a specific file and potentially add it to your bashrc file. If you have not done so you will need to source the file to setup the catkin workspace. The following line can be added to the end of the ~/.bashrc file so that it is automatically ran everytime a new a shell is open for the user. 

```Bash
$ source /opt/ros/melodic/setup.bash
```

If a different version of ROS melodic was installed you would replace melodic with distro of the ROS version that is installed.
```Bash
$ source /opt/ros/<distro>/setup.bash
```

### Catkin Workspace Setup
In order to develop new ROS packages and write code that performs a specific function, we will need to create a workspace to use as development. ROS using catkin as the build system for ROS. If you would like more information how Catkin works see the following http://wiki.ros.org/catkin/conceptual_overview. 

We need to create a workspace so in this example we will call our workspace catkin_ws. We will also create a folder called src where all of the source code for our packages will be. The full guide of creating a workspace can be found here http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment. 

The following is the essntial commands needed:
```Bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

After running catkin_make we should see build and devel folders next to src inside of workspace. In order to run any built packages with rosrun and roslaunch we will need to source a setup file built during catkin_make:

```Bash
source devel/setup.bash
```

Once the workspace is set up, I reccomend going through the ROS tutorials to familiarlize yourself with how everything works system works. 

## Docker ROS
Using docker and ros is a good idea for running testing and integration environments. It is also good for running ROS on linux distrubitions such as Arch and Gentoo that don't have native installs or are difficult to get up and running without the proper experience. You can also run graphhical applications in docker containers and share an x session with the host computer, run a x server within the container and connect with vnc, or do x forwarding over ssh from the container.
### Docker Setup
Install docker, start the daemon, and start the daemon on system boot:
```Bash
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
```

If you want to avoid typing sudo whenever you run the docker command, add your username to the docker group:

```Bash
sudo usermod -aG docker ${USER}
```

More information about docker and be found at the following links:
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04

https://wiki.archlinux.org/index.php/Docker

https://docker-curriculum.com

There are other sites out there like the official docker documtation. It is a bit advanced in going into kubernetes and other management systems of docker containers but there may be some good information in there:

https://docs.docker.com/engine/docker-overview/

### Docker and VS Code
Visual Studio Code has some nice ways of handling a docker container and mounting a specific folder within for a development environment. You will have to use the official proprietary version of of Visual Studio Code to work the the container plugin. The OSS version does not have all the features available for the plugin.

The following guide wasn't to hard to follow:
https://amirdarwesh.com/posts/2019/09/13/ROS-Docker-Vscode/

Only changed the DockerFile:
```Docker
FROM osrf/ros:melodic-desktop-full-bionic

RUN apt-get update && apt-get install -q -y \
    openssh-client tmux vim

RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc
```

VNC example that runs Gazebo:
https://github.com/devrt/ros-devcontainer-vscode

### Catkin Workspace Setup
Once you get a shell opened up in the container do the the above. The guide from amirdarwesh has a catkin_ws folder in the users home directory on the host system get mounted in the container. So the only thing necessarry to perform in int he container shell is to initialize the workspace and make sure a src directory is built and source the devel folder after the catkin make.

# Developing ROS Packages
