# Week 10-11 Instructions

## Introduction
M5 will see you integrating all the components that you have been developing so far and improving on the performance of this integrated system. You will demonstrate this integrated system in Week 11, and M5 will also serve as a trial run of the final demo and marking process. Before the live demo in Week 11, you will need to submit a copy of your implementation on Moodle. At the start of your Lab Session you can download your submission, and you will need to unzip it right before your turn to demo. Week 11 Lab Sessions will be recorded by the demonstrators.

During the live demo in Week 11 (and the final demo in Week 12), the task is for the robot to roam through an arena, which has a number of ARUCO markers on the walls, as well as sheep figurines and coke cans at different locations of the arena. The robot will need to generate a map of this arena, which gives the (x,y) coordinates of each of the ARUCO markers, sheep figurines, and coke cans it has found. You can either teleoperate the robot or let the robot drive autonomously. Each team has at most 5 minutes to set things up before starting the demo, and can spend at most 10 minutes (simulator world clock) in the arena. If you are confident with the resulting map you can end the demo earlier. While the next group is setting things up the previous group will need to send the resulting map of their demo run to the demonstrator for marking. If the demonstrator haven't received your map by the end of your lab session your [performance mark](#Performance-marking-scheme-55pts) will be 0, and you will only get the [live demo mark](#Live-demo-marking-scheme-25pts) for M5.

For M5, both the live demo and the resulting outputs will be graded. The M5 marking arena will be provided an hour before each lab session, i.e., there are 3 versions of the M5 marking arena, one for Lab Session 1, one for Lab Session 2, one for Lab Session 3. These three maps will have slight variations in their layouts. The marking arena world file for each lab session will be released at the start of each lab session in Week 11.

For Lab Session 1 (15 groups) and Lab Session 2 (14 groups) there will be two breakout rooms, each managed by one demonstrator; for Lab Session 3 (5 groups) there will be one room. The groups will demo in random ordering, and if there is a connectivity issue the group can move to demo later. If the connectivity issue can't be resolved during the sesion the demonstrator will run the group's implementation on their machine instead. The group demoing their solutions will share their screen with the rest of the room. The demonstrators will not answer questions related to implementation or performance during the live demo. 

Week 11 will be a trial run of the final demo and marking process. M5 will be marked following the same marking scheme as the final demo, except for in the final demo there is an additional written report to be submitted after the live demo. The marking arenas for each lab session and for M5 and final demo will have slight variations, but for all arenas the robot will start from (0,0) and will have at least 2 ARUCO markers in sight at the start.

## Objectives
- Week 10: finish integrating your system so that it can navigate through an arena, locate all ARUCO markers, sheep figurines, and coke cans, and generate a map of this arena which gives the (x,y) coordinated of these targets.
- Week 11: live demo for M5 marking, which also serves as a trial run of the final demo and marking process

## Marking schemes
As no written report is needed for M5, the total mark for M5 is 80pts. You resulting mark will be mutiplied by 1.25 to scale to 100pts. For example, if you get 25pts for your live demo mark and 50pts for your performance mark, then your M5 score will be (25+50)*1.25 = 93.75

### Live demo marking scheme (25pts)
- Robot control (15pts at most) 
  - The robot drives through the arena entirely by teleoperation: 5pts
  - The robot drives through the arena partially autonomously, that is, the robot is able to autonomously drive and locate targets, but needs manual intervention during its run through the arena. For example, you may have the robot automatically explore part of the arena and then manually move it to another part of the arena and set it to automatically explore again: 10pts
  - The robot drives through the arena fully autonomously without any manual intervention from start till end: 15pts
- Task (4pts)
  - The robot finds at least one ARUCO marker within 10 minutes (1pts)
  - The robot can estimate the (x,y) coordinates of at least one ARUCO marker within 10 minutes that is within 2m (Euclidean distance) of the actual pose (1pts)
  - The robot finds at least one sheep figurine or coke can (1pts)
  - The robot can estimate the (x,y) coordinates of at least one sheep figurine or coke can within 10 minutes that is within 2m (Euclidean distance) of the actual pose (1pts)
- Time (6pts):
  - 3pts if the robot finds all the targets (ARUCO marker, sheep, coke) and finishes generating the map within 10 minutes (simulator world clock)
  - 6pts if the robot finds all the targets and finishes generating the map within 5 minutes (simulator world clock)
### Performance marking scheme (55pts)
- Finding targets (20pts)
  - Finding ARUCO markers (0-5pts): each ARUCO marker found +0.5pt
  - Finding coke cans (0-5pts): each coke can found +1pt
  - Finding sheep figurines (0-10pts): each sheep figurine found +2pt
- Estimating poses (35pts)
  - Generate a csv file containing the (x,y) coordinates of the ARUCO markers (with correct ID of each marker), sheep figurines (ID doesn't matter), and coke cans (ID doesn't matter) in the format of the [testing arena true map](TruePose_demo_arena_dev.csv) 
  - ARUCO markers' (x,y) coordinates (0-10pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of an ARUCO marker +0.5pt
  - Coke cans' (x,y) coordinates (0-10pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of a coke can +1pt
  - Sheep's (x,y) coordinates (0-15pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of a sheep figurine +1.5pt

## Getting-started
### Load the development arena for testing your integrated system
1. Extract [fence.zip](fence.zip) to your ```catkin_ws/src/penguinpi_gazebo/models``` directory. This is the model of a 12m x 12m fence for the arena
2. Copy [demo_arena_dev.world](demo_arena_dev.world) to your ```catkin_ws/src/penguinpi_gazebo/worlds``` directory, and [demo_arena_dev.launch](demo_arena_dev.launch) to your ```catkin_ws/src/penguinpi_gazebo/launch``` directory
3. Launch the arena by running the following commands inside the terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch penguinpi_gazebo demo_arena_dev.launch 
```
The marking arena for M5 and for final demo has the same 12m x 12m fense, with slight variations in target locations and postitions of the interior walls.

### Estimating locations of the sheep and coke
You can estimate the location of the sheep figurine and the coke can by measuring their size in the robot's camera view (you may use [nn_detect.py](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week06-07/nn_detect.py) and the model you trained in M3 for this), or by estimating the location relevant to nearby ARUCO markers and their estimated pose.

To help you estimate the pose of the sheep figurine and the coke can, below are their dimensions (the unit is meter, this info can be retrieved using Gazebo's BoundingBox function):
- the sheep's dimension is x=0.108, y=0.223, z=0.204 (original size is 10 times this but in the sdf file there is a 0.1 scale factor)
- the coke's dimension is x= 0.06, y=0.06, z=0.14

![The development arena](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/DevArena.png?raw=true "The development arena for testing your integrated system")
