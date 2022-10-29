# Robotic Manipulator for Educational Purposes

In the folder description are the mesh files and urdf file for robot description.

In the folder  package manipulator_planning the IK of the arm is implemented using TRAC-IK.

In the folder  package manipulator_vision are some files using the Jevois camera and the data from the camera to transform coordinates of a object and try to calculate the IK.

In the  folder manipulator_hw is a hardware interface used to control the robot and send/receive command positions.

In the folder CAD are the editable files of the robot design in Solid Works.



All the materials necessary to recreate the use of this and control are in the directory. Note that the motors were actuated by a PWM driver and the master running ROS was conected to the raspberry via network using a ethernet cable.
