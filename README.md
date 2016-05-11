# Third year final project - The Robot Shopkeeper

This is my third year project, aiming to develop an integrated robotics system using a Baxter robot to allow Baxter to be able to run a sweet shop and respond to customers.

The code provided will is separated into different catkin packages, so that each section of the software can be looked at/run individually if needed. They are separated into the main section in the /src folder: manipulation, perception and interaction.

[Interaction](https://github.com/um10kh/baxter-project/tree/master/src/interaction/src) includes the code for locating customers and the server for the voice recognition Android app.

[Perception](https://github.com/um10kh/baxter-project/tree/master/src/perception/src) includes code for the .cpp files used to analyse the Kinect pointclound to find the bowl and the main vision files used for locating and analysing the sweets on the table.

[Manipulation](https://github.com/um10kh/baxter-project/tree/master/src/manipulation/src) includes the main code for the overall system, which integrates movement along with the other developed features.

## Installation

First of all, following ROS tutorials, create a catkin workspace to add packages into. Once in the catkin workspace, ```git clone``` the repository into that directory so you have the interaction, perception and manipulation packages available to you.

## Materials

To set up the shop for Baxter, you need five main items. 

1. Firstly, in this project, blue, red and green Quality Street sweets were used. Whilst similar coloured sweets may work in place of this, the colour recognition may not be as accurate for them.

2. Secondly, you need a small, white polystyrene bowl to hold the sweets, available in most shops/supermarkets. In this bowl, place 4 green, 4 blue and 4 red sweets.

3. Thirdly, a piece of A2 paper (or two A3 pieces stuck together) is needed, with a thick black border drawn around it. This paper needs to be stuck in the centre of the table, the closer Baxter, the more preferrable.

4. Next, a container is needed for Baxter to drop the sweets into, one large enough that he can hold his gripper over them and drop from a height. It may be an idea to weight the bag slightly so it doesn't move about the table once placed.

5. An Android device is needed, which will have the voice recognition application installed on it, discussed later on.

![](https://github.com/um10kh/baxter-project/blob/master/readmeimages/bowl.jpg) ![](https://github.com/um10kh/baxter-project/blob/master/readmeimages/bag.jpg) 

![](https://github.com/um10kh/baxter-project/blob/master/readmeimages/paper.jpg)![](https://github.com/um10kh/baxter-project/blob/master/readmeimages/orientation.jpg)

To orient these items, the bowl should be somewhere to the left of the page (if you aren't using a Kinect, you can place it in position after running). The page should be placed in the middle and the sweet container can be anywhere within Baxter's right gripper's reach. In practice, it seemed easy to place it on the right of the page.

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

3. After that, place the left gripper on level with the table 

4. Finally, place the right gripper above wherever the customer's sweet container is, then press enter.



After these tasks, stand away from the camera and the system will run. From then a customer can approach Baxter's camera to start the interaction.

### With Kinect

To then run the system with the Kinect, launch the alternative roslaunch file:

```roslaunch manipulation shopkeeper.launch```

This should launch the whole system. Make sure none of the nodes display any errors in the terminal at this point, aside from the camera commands may display an error (depending on which two of Baxter's cameras are currently open).

Then, on screen, you can follow the commands (printed on the terminal) in order:

1. Make sure you open the Android app on the device (make sure it is fully closed first), and enter the IP address shown in the terminal.

2. Secondly, place the left gripper on level with the table 

3. Lastly, place the right gripper above wherever the customer's sweet container is, then press enter.

After these tasks, stand away from the camera and the system will run. From then a customer can approach Baxter's camera to start the interaction.
