# MIE 443 - Contest 2 Deliverable

Date: March 23, 2021
Group: 20

## Team:

|       Name       | Student Number |
|:----------------:|:--------------:|
|   Stefan Albers  |   1003476204   |
| Mithun Jothiravi |   1002321258   |
|   Osvald Nitski  |   1002456987   |
|    David Rolko   |   1003037420   |2

## Running Group 20 Contest 2 Submission

Open three terminals. Navigate to `catkin_ws` in each of them.

1. In terminal #1 run `roslaunch mie443_contest2 turtlebot_world.launch world:=practice`

2. In terminal #2 you will need the filepath to the map `.yaml` file, here we show the full path on Osvald's VM, change the path to match your directory structure. Run `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/<REPLACE THIS WITH YOUR USER SYSTEM NAME>/catkin_ws/src/mie443_contest2/maps/map_practice.yaml`

3. In terminal #3 run `rosrun mie443_contest2 contest2`

All the output results can be found in the `Output_file.txt` file located in the `catkin_ws`.

## Repository:

Entire project repo can be accessed [here](https://github.com/OsvaldN/MIE443_Contest2/tree/nav_osvald).
