# Week 12 Instructions

## Introduction
This is your final assessment of the course, which contributes to 60% of your total score.

Before the live demo in Week 12, you will need to submit a copy of your implementation on Moodle. At the start of your Lab Session you can download your submission, and you will need to unzip it right before your turn to demo. Week 12 Lab Sessions will be recorded by the demonstrators.

During the final demo in Week 12, the task is for the robot to roam through an arena, which has a number of ARUCO markers on the walls, as well as sheep figurines and coke cans at different locations of the arena. The robot will need to generate a map of this arena, which gives the (x,y) coordinates of each of the ARUCO markers, sheep figurines, and coke cans it has found. You can either teleoperate the robot or let the robot drive autonomously. Each team has at most 5 minutes to set things up before starting the demo, and can spend at most 10 minutes (simulator world clock) in the arena. If you are confident with the resulting map you can end the demo earlier. While the next group is setting things up the previous group will need to send the resulting map of their demo run to the demonstrator for marking. If the demonstrator haven't received your map by the end of your lab session your [performance mark](#Performance-marking-scheme-55pts) will be 0, and you will only get the [live demo mark](#Live-demo-marking-scheme-25pts) and the [report mark](#Written-report-marking-scheme-20pts).

For Lab Session 1 (15 groups) and Lab Session 2 (14 groups) there will be two breakout rooms, each managed by one demonstrator; for Lab Session 3 (5 groups) there will be one room. The groups will demo in random ordering, and if there is a connectivity issue the group can move to demo later. If the connectivity issue can't be resolved during the sesion the demonstrator will run the group's implementation on their machine instead. The group demoing their solutions will share their screen with the rest of the room. The demonstrators will not answer questions related to implementation or performance during the live demo. 

Final demo will be marked based on your live demo performance and an additional written report. The marking arenas for each lab session will have slight variations, but for all arenas the robot will start from (0,0) and will have at least 2 ARUCO markers in sight at the start. The marking arena world file for each lab session will be released at the start of each lab session in Week 12.

Within a week of the final demo, you will need to submit a written report reflecting on your implementation. The report should be at most 4 pages long (minimum margin 0.5cm, minimum font 12pt). It should include:
- A function you implemented that you are most proud of and its design process
- Functions you implemented that did not work as expected during the demo, why they did not work as expected, and how to fix them in the future
- Parts of your implementation that can be improved, and how to improve them
- Additional functions that can be added in the future, either inspired by other groups' demo or by the course contents, and what benefits they may bring

## Objectives
Final demo and competition

## Marking schemes
### Bonus point for the top-5 teams (1% bonus mark added to the total course score)
After the live demos, all teams will be ranked, first by how many targets (ARUCO marker, sheep, coke) were found, then by how accurate the estimated location of the targets are (the average Euclidean distance between estimated location and true location of all targets), finally by how fast they were at mapping the arena (simulator world clock). The top-5 teams will receive a 1% bonus mark added to their total course score.

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
  - Generate a csv file containing the (x,y) coordinates of the ARUCO markers (with correct ID of each marker), sheep figurines (ID doesn't matter), and coke cans (ID doesn't matter) in the format of the [testing arena true map](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week10-11/TruePose_demo_arena_dev.csv) 
  - ARUCO markers' (x,y) coordinates (0-10pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of an ARUCO marker +0.5pt
  - Coke cans' (x,y) coordinates (0-10pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of a coke can +1pt
  - Sheep's (x,y) coordinates (0-15pts): each estimated x OR y coordinate that is within 1m from the true x OR y coordinate of a sheep figurine +1.5pt
### Written report marking scheme (20pts)
- Describe the design decisions of a function that you are most proud of (4pts)
- Functions that did not work as expected (4pts in total)
  - Identifying at least one function you implemented that did not work as expected during the demo (1pts) 
  - Identifying cause of this failure (1pts) 
  - Identifying how to fix this function in the future (2pts)
- Possible improvements (6pts in total)
  - Specifying at least one part of your implementation that can be improved (2pts)
  - Identifying how to improve it (2pts) 
  - Specifying a second part that can be improved (1pt)
  - Identifying how to improve this second part (1pt)
- Possible additions (6pts in total)
  - Specifying at least one function that can be added in the future (2pts)
  - Identifying what benefits it may bring (2pts)
  - Specifying a second function that can be added (1pt)
  - Identifying benefits of this second additional function (1pt)

## Getting-started
- Improve on what you've observed in M5
- the sheep figurine's size is x=0.108, y=0.223, z=0.204; the coke can's size is x= 0.06, y=0.06, z=0.14.
