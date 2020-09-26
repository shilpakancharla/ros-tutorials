# Tutorials with Robot Operating System (ROS)

## Ubuntu Install for ROS Melodic

For OSX, use a virtual machine VirtualBox version 5.2.44 (or any version that is being maintained currently). Set up the virtual machine using the instructions at https://brb.nci.nih.gov/seqtools/installUbuntu.html. 

Install Ubuntu 18.04.5 LTS (Bionic Beaver): https://releases.ubuntu.com/18.04/. Get the desktop image.

Open terminal on VM and run 

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full
```

Add the following the bashrc: ```source /opt/ros/melodic/setup.bash```

## C++ Basics

* Structs only contain variables

* Classes contain variables and functions

* Place functions of the class out of class to keep class short
