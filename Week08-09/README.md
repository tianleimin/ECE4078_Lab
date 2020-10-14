# Week 8-9 Instructions
- [Introduction](#Introduction)
- [Objectives](#Objectives)
- [Marking schemes](#Marking-schemes)
- [Getting-started](#Getting-started)
    - [Implement a basic auto mapping and navigation (week 8)](#Fill-in-missing-segments-to-implement-a-basic-auto-mapping-and-navigation-week-8)
    - [Improve performance of auto mapping and navigation (week 9)](#Improve-performance-of-auto-mapping-and-navigation-week-9)

## Introduction
In M2 you used teleoperation to implement SLAM. The goal of M4 is to implement autonomous mapping and navigation. 

You will use the [same testing arena](penguinpi_arena.world) as M2 and M3 (see [Week03-05's instructions](https://github.com/tianleimin/ECE4078_Lab/tree/master/Week03-05#Launch-the-cardboard-arena-world-week-3) for how to launch the testing arena). You will also need [penguinPiC.py](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week01-02/penguinPiC.py), as well as the [wheel calibration](https://github.com/tianleimin/ECE4078_Lab/tree/master/Week03-05/calibration/wheel_calibration) and [camera calibration](https://github.com/tianleimin/ECE4078_Lab/tree/master/Week03-05/calibration/camera_calibration) information for running auto nav.

The starter code [autonav_starter.py](autonav_starter.py) provides a basic implementation of auto mapping and navigation which follows the steps below:

1. The robot starts at location (0,0). It rotates 360 degrees and stores the estimated (x,y) locations of all the ARUCO markers it has seen during spinning. **Our assumption is that seeing a marker means there is a path to that marker without any obstacles in between.** The robot now has a map of which markers the starting point (0,0) leads to and the associated distance (for both the testing arena and the marking arena two markers will be visible from the starting point). 
2. The robot selects the closest marker A and drives to that marker.
3. Once the robot arrives at marker A, it rotates again and stores the markers it has seen at this new location and their estimated (x,y). The robot now expands the map one level deeper and knows which markers it can reach from marker A and the associated distances.
4. This process is repeated until all markers are found or until time-out. The robot now has a map of ARUCO markers and the connecting paths between them with the associated distances.

[TestArenaTrueMap.txt](TestArenaTrueMap.txt) provides the true pose of all markers in the testing arena, as well as paths between marker pairs. You may use this to evaluate the performance of your implementation. 

**You may implement your own auto mapping and navigation without using the starter code, as long as your implementation generates a map in the same format as [TestArenaTrueMap.txt](TestArenaTrueMap.txt).**

## Objectives
Implement an auto mapping and navigation function that locates ARUCO markers in an area and identifies paths between them.

## Marking schemes
Your implementation will be executed in a marking arena, which has a different layout than the testing arena. In the marking arena the robot will still start at (0,0) and there will be three markers visible at the starting point. There are 8 markers in total for the marking arena.

1. Number of markers found (16pts): each marker found +2pts
2. Estimated (x,y) pose of markers (16pts): each estimated x OR y of a marker within 1m of the true x OR y +1pts (if multiple estmations are given for the same marker the estimation closest to the true pose will be used for marking)
3. Number of paths found (24pts): each path found +2pts until reaching the maximum 24pts. *If you simply generate a fully connected graph without having the robot drive in the arena you will lose all 24pts as a penalty.*
4. Estimated path distance (24pts): each estimated distance of a path less than 1m off the true distance +2pts until reaching the maximum 24pts (if multiple estimations are given for a path's distance the estimation closest to the true distance will be used for marking)
5. Time spent generating the map (20pts): 0pt if mapping takes 20 minutes or longer; 5pts if mapping takes 15-20 minutes; 10pts if mapping takes 10-15 minutes; 15pts if mapping takes 5-10 minutes; 20pts if mapping takes less than 5 minutes. The task time will be normalized by the Gazebo frame rate as normalized_time = time * framerate / 30. For example, if you use a native Ubuntu with GPU that has a framerate of 90fps, and spend 3 minutes for the task, your normalized time will be 9 minutes; if you use a VM with a framerate of 15fps, and spent 8 minutes for the task, your normalized time will be 4 minutes.

## Getting-started
### Fill in missing segments to implement a basic auto mapping and navigation (week 8)
1. Calculate the time the robot needs to spin 360 degrees at [line 62](autonav_starter.py#L62)
2. Calculate the time the robot needs to spin and drive towards a marker at [line 145](autonav_starter.py#L145)

### Improve performance of auto mapping and navigation (week 9)
Below are suggested improvements that you can start with:

1. Improve the pose estimation of the ARUCO markers at [line 112](autonav_starter.py#L112)
2. Improve the search strategy for expanding the map at [line 125](autonav_starter.py#L125)
3. Improve the pose estimation of the robot at [line 156](autonav_starter.py#L156)
4. Including visual feedback. For example, when the robot is spinning a 360 to find visible markers, in addition to calculating the time the robot needs to rotate based on the motion model, seeing the same marker twice can also be used as a cue that the robot has finished a 360. Similarly, when travelling to a found marker, you can use visual feedback to adjust the robot's pose to keep the marker centered in its view insead of only relying on the calculation of spin time and drive time.

You may implement your own auto mapping and navigation without using the starter code as long as it generates a map in the same format as [TestArenaTrueMap.txt](TestArenaTrueMap.txt).
