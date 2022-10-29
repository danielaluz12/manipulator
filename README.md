# Robotic Manipulator for Educational Purposes
The robot can be seen in Rviz:

![pose3_end_rviz](https://user-images.githubusercontent.com/71453516/198848489-23eef356-f268-4bc0-80fc-f1034f7dbbed.png)

And here where it is actually built:

![pose3_real_end](https://user-images.githubusercontent.com/71453516/198848527-6832e4d2-73eb-4607-a53f-abb904a4dd80.png)

In the folder description are the mesh files and urdf file for robot description.

In the folder  package manipulator_planning the IK of the arm is implemented using TRAC-IK.

In the folder  package manipulator_vision are some files using the Jevois camera and the data from the camera to transform coordinates of a object and try to calculate the IK.

In the  folder manipulator_hw is a hardware interface used to control the robot and send/receive command positions.

In the folder CAD are the editable files of the robot design in Solid Works.

All the items used for control and to move the arm can be seen below:

![pwm_raspberry](https://user-images.githubusercontent.com/71453516/198848591-f38bfdb6-f24c-4678-9b46-ed2d8a65332b.jpg)

raspberry(1), PWM Driver(2) e servos(3)

All the materials necessary to recreate the use of this and control are in the directory. Note that the motors were actuated by a PWM driver and the master running ROS was conected to the raspberry via network using a ethernet cable.
