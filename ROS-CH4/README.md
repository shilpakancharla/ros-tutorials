# Chapter 4: Kickstarting Robot Programming using ROS

## What is Robot Programming?

* Robot is a machine with sensors, actuators (motors), and a computing unit that behaves based on user controls, or it can make its own decisions based on sensors inputs.

* Brain of a robot can be a microcontroller or a PC.

* __Robot programming__: process of making the robot work from writing a program for the robot's brain.

* __Actuators__: move a robot's joints, providing rotary or linear motion.

* __Sensors__: provide robot's state and environment.

* Actuators are controlled by motor controllers and interface with a microcontroller/PLC (programmable logic controller). Sensors also interface with a microcontroller or PC.

## Why Robot Programming is Different

* The differences between robot programming and conventional programming are the input and output devices. The input devices include robot sensors, teach pendants, and touch screens, and the output devices nclude LCD displays and actuators. 

### Features needed for programming robots

* __Threading__: May require a multi-threaded compatible programming language in order to work with different sensors and actuators in different threads. This is called __multitasking__. Each thread can communicate with each other to exchange data.

* __High-level object-oriented programming__: OOP languages are more modular and code can easily be reused. Code maintenance is easy compared to non-OOP languages.

* __Low-level device control__: HLPL can also access low-level devices such as GPIO pins, serial ports, USB, SPI, and I2C. Programming languages like C/C++ and Python can work with low-level devices.

* __Ease of programming__: Python is a good choice in prototyping robot algorithms quickly.

* __Interprocess communication__: We can use multithreading architecture or write an independent program for doing each task. This feature creates multiple programs intsead of a mutlithreading system (socket programming is an example of this.

* __Performance__: If we work with high-bandwidth sensors like depth cameras and laser scanners, the computing esources needed to process the data is high. A good programming language can only allocate appropriate computing resources without loading the computing resource. 

* __Community Support__: Ensure the programming language has a enough community support for that language.

* __Availability of third-party libraries__: Image processing - OpenCV; availability makes programming easier.

* __Existing robotics software framework support__: If your program has the support, it is easier to prototype the application.

## Getting Started with ROS

* __Message passing interface between processes__: ROS provides a message passing interface to communicate between two programs or processes. __Interprocess communication__ refers to two processes communicating with each other.

* __Operating system-like features__: ROS is not a real operating system. It is a meta operating system that provides some OS functionalities. These include multi-threading, low-level device control, package management, and hardware abstraction. The hardware abstraction layer enables programmers to program a device. The advantage is that we can write code for a sensor that works the same way with different vendors. We don't need to rewrite the code when we use a new sensor. Package management helps users organize software in units called packages. Each package has source code, configuration files, or data files for a specific task. These packages can be distributed and installed on other computers.

* __High-level programming language support and tools__: Supports popular languages like C++, Python, and Lisp. Experimental support for C#, Java, Node.js, etc. Flexibility allows programmers to spend less time in creating build systems for their applications.

* __Availability of third-party libraries__: ROS framework is integrated with most popular third-party libraries. OpenCV for robotic vision, PCL for 3D robot perception.

* __Off-the-shelf algorithms__: ROS has implemented popular robotics algorithms such as PID, SLAM; and path planners such as A*, Dijkstra, AMCL. This reduces the develpment time for prototyping the robot.

* __Ease in prototyping__: Packages can be reused for any robot. Most are open-source and resuable for commercial and reserach purposes. This can reduce development time.

* __Ecosystem/community support__

* __Extensive tools and simulators__: ROS has many CLI and GUI tools to debug, visualize, and simulate robotic applications.
