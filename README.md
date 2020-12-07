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

# Gazebo

* __World__: collection of robots and objects (such as buildings, tables, and lights), and global parameters including the sky, ambient light, and physics properties.

* __Static__: entities marked as static (those having the `<static>true</static>` element in SDF), are ojbects which only have collision geometry. All objects which are not meant to move should be maked as static, which is a performance enhancement.

* __Dynamic__: entities marked as dynamic (either missing the `<static>` element or setting false in SDF), are objects which have both intertia and a collision geometry. 

## Position Models

The pose of each model may be altered through the translate and rotate tools: 

### Translation

The translate tool allows you to move the object along the x, y, and z axes. Select this tool now (or press `t`) and click on the object you want to move. A three axes visual marker will appear over the object, which allows you to move the object in x, y, and z directions. 

* You can also just click on the object itself and drag it to move on the x-y plane. You may control which axis the object moves along by pressing and holding the `x`, `y`, or `z` key while dragging the object. 

* You can hold the `Ctrl` key to snap the movement to a 1 meter grid. 

* If the object is not aligned with the world (for example after you use the rotate tool explained next), you can hold the `Shift` key so the visual markers show up aligned with the world, and you can translate in world coordinates.

### Rotation

The rotate tool allows you to orient a model around the x, y, and z axes. Select this tool now (or press `r`) and click on the object you want to move. Three ring-shaped visual marker over the object, which allows you to rotate the object around the x, y, and z axes. 

* You can also just click on the object itself and hold the `x`, `y`, or `z` keys while dragging it to the constraint the motion to one of these axes. 

* You can hold the `Ctrl` key to snap the movmeent to 45 degree increments. 

* If the object is not aligned with the world, you can hold the `Shift` key so the visual markers show up aligned witht he world, and you can rotate about the world axes. 

### Scale

The scale tool allows you to resize a model in the x, y, and z directions. Currently the scale tool only works with simple shapes, i.e., box, cylinder and sphere. Select this tool now (or press `s`) and click on a simple shape. A three axes visual marker will appear over the object, which allows you to scale the x, y, and z dimensions of the object. 

* You can also just click on the object itself and hold the `x`, `y`, or `z` keys while dragging it to constrant the scaling to one of these axes. 

* You can hold the `Ctrl` key to scale in 1 meter increments. 

# SDFormat

SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control. Originally develped as part of the Gazebo robot simulator, SDFormat was designed with scientific robot applications in mind. Over the years, SDFormat has become a stable, robust, and extensible format capable of describing all apsects of robots, static and dynamic objects, lighting, terrain, and even physics. 

You can accurately describe all aspects of a robot using SDFormat, whether the robot is a simple chassis with wheels or a humanoid. In addition to kinematic and dynamic attributes, sensors, surface properties, textures, joint friction, and many more properties can defined for a robot. These features allow you to use SDFormat for both simulation, visualization, motion planning, and robot control. 

Simulation requires rich and complex environments in which models exist and interact. SDFormat provides the means to define a wide variety of environments. Multiple lights may be included in an environment, terrain (either fictional or based on a DEM), streets from OpenStreetMaps, and any model provided from The Prop Shop (on online repository of 3D models).
