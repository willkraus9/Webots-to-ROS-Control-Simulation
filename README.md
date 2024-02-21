# Webots-to-ROS-Control of an Autonomous Vehicle
Converting an autonomous vehicle controller in Webots into ROS

![Results](https://github.com/willkraus9/Webots-to-ROS-Control/blob/1a3ab11c03f13875878b7ed3bb3db38e4f918547/CMU_PID%20GIF.gif)

## Introduction

This project is an implementation of Carnegie Mellon’s 24-677 (Modern Control Theory) from Webots simulation to ROS Noetic and Gazebo. An autonomous vehicle navigates a racetrack using waypoints from a .csv file. Not only was this a fun project to finish over winter break, but this also serves as a great introduction to ROS for interested students. 

Since this was a class project, I have removed potential answers and left the ROS import shell. If you would like the full code, please send me an email (will.kraus9@gmail.com).

## ROS: Basic Resources
For this project, I used Oracle VMWare to simulate a Linux environment instead of dual booting my comptuer. You can find a tutorial about installing ROS using VMWare here: https://www.instructables.com/How-to-Install-ROS/ 

NB: I had an issue with using the sudo command when installing Linux in this way, solved with this tutorial: https://www.baeldung.com/linux/username-not-in-sudoers-file

Here's a great ROS introduction from Udemy (make sure to get the 7 day free trial): https://www.udemy.com/course/ros-for-beginners/

## Gazebo
I used the Clearpath Jackal as my model for the Gazebo simulation; install directions can be found here: https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html

NB: The starter world has too many obstacles, so you can use the empty_world.launch file instead: https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/additional_sim_worlds.html

## Test Files
The .launch file called check_odometry runs the experiment; there is a odometry checking Python file also listed that I used to ensure I was calling the correct topics. Also, I made a Python file to make sure that the controller was taking in the correct data about the simulation.
