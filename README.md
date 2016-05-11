# Third year final project - The Robot Shopkeeper

This is my third year project, aiming to develop an integrated robotics system using a Baxter robot to allow Baxter to be able to run a sweet shop and respond to customers.

The code provided will is separated into different catkin packages, so that each section of the software can be looked at/run individually if needed. They are separated into the main section in the /src folder: manipulation, perception and interaction.

[Interaction](https://github.com/um10kh/baxter-project/tree/master/src/interaction/src) includes the code for locating customers and the server for the voice recognition Android app.

[Perception](https://github.com/um10kh/baxter-project/tree/master/src/perception/src) includes code for the .cpp files used to analyse the Kinect pointclound to find the bowl and the main vision files used for locating and analysing the sweets on the table.

[Manipulation](https://github.com/um10kh/baxter-project/tree/master/src/manipulation/src) includes the main code for the overall system, which integrates movement along with the other developed features.

## Installation

First of all, following ROS tutorials, create a catkin workspace to add packages into. Once in the catkin workspace, ```git clone``` the repository into that directory so you have the interaction, perception and manipulation packages available to you.

## Materials

To set up the shop for Baxter

## User Setup/Running the System

Once you have all the files downloaded and installed, you can then either run the system with or without Kinect, depending upon the system.

Before you run the overall system, make sure Baxter is enabled and his arms are untucked using the command:

```rosrun baxter_tools tuck_arms.py -u```

### Without Kinect

To run the system without the Kinect, first of all, launch the roslaunch file:

```roslaunch manipulation shopkeeper_nk.launch```

This should launch the whole system. Make sure none of the nodes display any errors in the terminal at this point, aside from the camera commands may display an error (depending on which two of Baxter's cameras are currently open).

Then, on screen, you can follow the commands (printed on the terminal) in order:

1. Make sure you open the Android app on the device (make sure it is fully closed first), and enter the IP address shown in the terminal.

2. Next, place the bowl under the left gripper so that the left-centre side of the bowl is aligned in the centre of the gripper, then press enter on the keyboard to get the next command.

3. Lastly, place the left gripper on level with the table and the right gripper above wherever the customer's sweet container is, then press enter.

After these tasks, stand away from the camera and the system will run. From then a customer can approach Baxter's camera to start the interaction.

### With Kinect

To then run the system with the Kinect, launch the alternative roslaunch file:

```roslaunch manipulation shopkeeper.launch```

This should launch the whole system. Make sure none of the nodes display any errors in the terminal at this point, aside from the camera commands may display an error (depending on which two of Baxter's cameras are currently open).

Then, on screen, you can follow the commands (printed on the terminal) in order:

1. Make sure you open the Android app on the device (make sure it is fully closed first), and enter the IP address shown in the terminal.

2. Secondly, place the left gripper on level with the table and the right gripper above wherever the customer's sweet container is, then press enter.

After these tasks, stand away from the camera and the system will run. From then a customer can approach Baxter's camera to start the interaction.
