# Week 12 Instructions

## Introduction
This is your final assessment of the course, which contributes to 60% of your total score.

There are two parallel tracks for final demo on 9 Nov between 9am-12pm and 1pm-7pm, and you can attend either one of them. To book a 30min slot, please go to [Room 1](https://calendar.google.com/calendar/selfsched?sstoken=UUNYUWk5ekt0Nm9afGRlZmF1bHR8OGEwZjU3ZGI4MDk4N2Y5OTZkMjZhMTUzMWZkN2U0ZDQ) or [Room 2](https://calendar.google.com/calendar/selfsched?sstoken=UUl0N3J0WDNRelh5fGRlZmF1bHR8YmM1ODAwMTc5MjJiMGE2MDhmZTU5NWE5YjJhMzcxYmE)'s appointment page. When making an appointment, please specify your group number as shown in the example below. **Each group can only book one slot with one room. If a group has made multiple bookings all of their bookings will be cancelled and they will need to book again.**

![Make an appointment for final demo](FinalDemoBooking.png?raw=true "Specify your group number when booking an appointment for final demo")

Before 9am on Mon 9 Nov, you will need to submit a copy of your implementation on Moodle. At the start of your demo session you will need to download your submission and unzip it for your demo. After running the live demo you will need to submit the generated map on Moodle during your demo session. If the map hasn't been submitted by the end of your demo session your [performance mark](#Performance-marking-scheme-55pts) will be 0. Final demo will be recorded by the demonstrators.

During the final demo, the task is for the robot to roam through an arena, which has a number of ARUCO markers on the walls, as well as sheep figurines and coke cans at different locations of the arena. The robot will need to generate a map of this arena, which gives the (x,y) coordinates of each of the ARUCO markers, sheep figurines, and coke cans it has found. You can either teleoperate the robot or let the robot drive autonomously. Each team has at most 5 minutes to set things up before starting the demo, and can spend at most 10 minutes (simulator world clock, roughly equal to 20min real world time) in the arena. If you are confident with the resulting map you can end the demo earlier.

We will set up 30min slots for final demo. There will be two parallel sessions between 9am-12pm and 1pm-7pm on 9 Nov, each managed by two demonstrators. You can register to demo in one of these slots. If there are unsolvable issues that prevent you from performing the live demo during your assigned slot, a demonstrator will run your submitted implementation on their machine and mark your performance based on the run on their machine.

Final demo will be marked based on your live demo performance and an additional written report. A final demo arena will be provided at 9am on Mon 9 Nov, in which the robot will start from (0,0) and will have at least 2 ARUCO markers in sight at the start.

Before 6pm Fri 13 Nov, you will need to submit a written report reflecting on your implementation. The report should be at most 4 pages long (minimum margin 0.5cm, minimum font 12pt). It should include:
- A function you implemented that you are most proud of and its design process
- Functions you implemented that did not work as expected during the demo, why they did not work as expected, and how to fix them in the future
- Parts of your implementation that can be improved, and how to improve them
- Additional functions that can be added in the future, either inspired by other groups' demo or by the course contents, and what benefits they may bring

For a video guide of the live demo process please see below:
[![Live demo process](https://img.youtube.com/vi/N-dMKyidO4k/maxresdefault.jpg)](https://youtu.be/N-dMKyidO4k)

## Objectives
Final demo and competition

## Marking schemes
### Bonus point for the top-5 teams (1% bonus mark added to the total course score)
After the final demos, all teams will be ranked, first by how many targets (ARUCO marker, sheep, coke) were found, then by how accurate the estimated location of the targets are (the average Euclidean distance between estimated location and true location of all targets), finally by how fast they were at mapping the arena (simulator world clock). The top-5 teams will receive a 1% bonus mark added to their total course score.

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

## FAQs
1. In your implementation and submission, the actual location of the targets retrieved from Gazebo must not be called anywhere. 
2. You will need to automatically classify targets. Manual object classification (manually entering the correct name of an object to be classified) is not permitted. You CAN manually switch the object detection component on or off (with your neural network model then automatically detecting which object the robot sees and estimating its pose), which counts as partial manual robot control (10pts). You CANNOT manually enter annotations of camera images, such as telling the robot which part of an image is a sheep by manually putting a bounding box around the sheep, or manually capture the current location of the robot as the location of the object that the robot is right next to.
3. For the same target you can provide at most 3 estimations in the generated map (you can make as many estimations during the run as possible, this is only limiting how many estimations you can print in the map that you submit for marking).
4. There will be no more than 7 sheep, 7 coke cans, and 15 ARUCO markers in each of the marking map. Thus, you can only provide estimations for no more than 7 sheep, 7 coke cans, and 15 ARUCO markers in the generated map.
5. No sheep or coke cans will be in the paths between two ARUCO markers in the marking map, similar to [demo_arena_dev_no_collision.world](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week10-11/demo_arena_dev_no_collision.world).
6. The timer only stops when your map is finalized (post-processing time will be included in your run time as well, such as selecting which 3 estimations to keep for an object).
7. You cannot get the ARUCO marker IDs from the Gazebo model's name or a pre-defined list of markers.
8. Inside ```catkin_ws/src/penguinpi_gazebo/scripts/server```, you can only change the server port section for resolving the connection issue (line 399 to end), do not make any other changes to it. 
9. If you have modified ```penguinpi.sdf```, make sure to include it in your submission file. You will need to download and copy-paste your customised sdf file into your catkin_ws during the live demo.
10. Please download the final demo arena world file and launch it during your live demo.
