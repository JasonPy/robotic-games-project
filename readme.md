# Robotic Games Project 2021
This repository includes a ROS package that handles the logic of a **mouse** playing the game catch. For this game please refer to [this repository](https://github.com/holger-ziti/RG2021_projects).

## Installation
Clone this package into your `catkin_ws/src` folder. Then execute `catkin_make`. 

## Configuration of catch/launch/map_1.launch
Replace `<!-- <include file="$(find group_x)/launch/team_mouse.launch"/> -->` by `<include file="$(find mouse)/launch/team_mouse.launch"/>`
