# Week 1 Lab: Teleoperation Instructions

## Controls

- Up Arrow: Forward

- Down Arrow: Reverse

- Left Arrow: Turn Left 

- Right Arrow: Turn Right 

- B key (Toggle): Boost in velocity

- Spacebar: Stop the bot

## Setup

First, source the catkin workspace by using the following commands.

```sh
    source ~/catkin_ws/devel/setup.bash
    roslaunch penguinpi_gazebo penguinpi.launch
```

Then, run the keyboard teleoperation script.

```sh
    cd Week01_team_1_04/
    python3 keyboardControlStarter.py
```
