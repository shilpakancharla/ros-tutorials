# What is ROS?

* A toolkit to help you build robots

* Primarily a message passing service that helps standardize communications across different pieces of a robot

* Since the messages are standard, it also provides many pre-built libraries and tools that interface with this system so you can take advantage of years of community work.

# How to organize code?

* In many software projects OOP is used to allow for different sections of code be isolated and reused.

* ROS does not rely on this, and instead relies on software being written as many programs that communicate with one another.

# Interprogram communication

## OS Provided communicated

* Different everywhere

* Only works on one system

## ROS communication

* Standard

* Flexible

* Easy to program

# Terms

* A __node__ is a program within ROS and is the most basic unit of code storage.

* A node can be spawned (started) by ROS and it can communicate with other nodes.

* A __topic__ is a communication channel between nodes. Think radio stations: one node broadcasts, other can choose to listen in.

* A topic can only send out one TYPE of message (i.e., string, numbers, etc.)

* A node can publish many topics and listen to many topics.

* A __message__ is a single piece of information sent across a topic (i.e., one number, one string, etc.)

* A __package__ is a collection of related nodes that are compiled together and have shared dependencies.
