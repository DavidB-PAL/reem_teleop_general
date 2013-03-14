ROS Node: reem_teleop_general
-----------------------------------
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>

A joystick tele-operation controller for the REEM robot (Gazebo simulation or hardware).<br>
Uses a joystick to control the arms (currently only joint-based control of left arm), hand, mobile base and head.

This node is very useful for testing other systems, where you need to move parts of the robot into a certain position.

Tested with Logitech joypad & ROS Fuerte. The button assignments will be different for XBox/PS3 joypad.<br>
Press & hold one of the 4 front buttons to select the control mode: base/head/left-arm/right-arm.<br>
In base mode, the 2 joysticks are used to drive forward/backwards & turn.<br>
In head mode, the 2 joysticks are used to pan & tilt the head.<br>
In left-arm mode, press the the lower-right button to toggle between joint-control/IK-control/disabled. The 2 joysticks + 1 joypad control the joints, plus 1 of the buttons moves the 7th axis in one direction. Two buttons on the right open/close the hand.

ToDo: Fix joint limits, add right arm control, fix IK control of arms, add keyboard control like in original node.

Based on pr2_teleop_general.


**Required ROS packages:** <br>
joystick_drivers<br>
reem_simulation<br>
reem_kinematics<br>

**Usage:** <br>
Plug in Logitech joypad.<br>
$ roslaunch reem_gazebo reem_empty_world.launch <br>
$ roslaunch reem_teleop_general reem_teleop_joystick.launch <br>
