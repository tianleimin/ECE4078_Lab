# Week 3-5 Instructions
- [Introduction](#Introduction)
    - [ARUCO markers](#ARUCO-markers)
    - [SLAM (Simultaneous localization and mapping)](#SLAM-Simultaneous-localization-and-mapping)
- [Objectives](#Objectives)
- [Marking schemes](#Marking-schemes)
- [Getting-started](#Getting-started)
    - [Launch the cardboard arena world (week 3)](#Launch-the-cardboard-arena-world-week-3)
    - [Motion model (week 3)](#Motion-model-week-3)
    - [ARUCO marker detection (week 4)](#ARUCO-marker-detection-week-4)
    - [SLAM (week 5)](#SLAM-week-5)


## Introduction
### ARUCO markers
![An example of ARUCO markers](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/ARUCO_marker_example.png?raw=true "An example of ARUCO markers")

[ARUCO markers](http://www.uco.es/investiga/grupos/ava/node/26) are square fiducial markers introduced by Rafael Muñoz and Sergio Garrido. OpenCV contains a trained [function](https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html) that detects the ARUCO markers, which will be used in this project (the dictionary we used to generate the markers was ```cv2.aruco.DICT_4X4_100```). PenguinPi will be using these ARUCO markers as road-signs to help it map the environment and locate itself. 

You'll need to install cv2.aruco by typing the following commands in the terminal:
```
python3 -m pip install opencv-contrib-python==4.1.2.30 matplotlib
cd ~/catkin_ws/
catkin_make
```

Note: if you are unable to install opencv due to the error "No module named skbuild", run ```python3 -m pip install scikit-build```.

### SLAM (Simultaneous localization and mapping)
![Example output of SLAM](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/SLAM_map.png?raw=true "Example output of SLAM")

You will be implementing [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) with Extended Kalman filter, which uses the drive signals and the ARUCO markers to perform localisation and mapping.

## Objectives
- Week 3: [Motion model](#Motion-model-week-3)
- Week 4: [ARUCO marker detection](#ARUCO-marker-detection-week-4)
- Week 5: [Manual SLAM in the cardboard arena](#SLAM-week-5)

## Marking schemes
You will be developing a manual SLAM function using [drive signals](#Motion-model-week-3) and [ARUCO markers](#ARUCO-marker-detection-week-4) inside [the testing arena](#Launch-the-cardboard-arena-world). The M2 assignment submission on Moodle will be open in Week 5 and will be due in Week 6.

You can follow the provided skeleton code and fill in the missing segments, or make changes to the current implementation as it has a lot of room for improvements. You can even implement everything from scratch as long as your implementation of SLAM makes use of the drive signals and ARUCO markers, and produces a list of the ARUCO marker IDs with their estimated x-y coordinates.

For marking, the robot will follow a scripted trajatory to traverse a marking arena, which has a different layout than the testing arena. In particular, it will have a different number of ARUCO markers at different locations, and variations in the walls / figurine locations. 

You will be marked by [the SLAM map your function generates](#SLAM-week-5) for the marking arena:
- Mapping (40pts): number of ARUCO markers found in the marking arena. 40pts for finding all the markers with correct IDs, each one missed -5pts
- Localisation (60pts): locate the (x,y) coordinates of ARUCO markers in the marking arena. Every x OR y coordinate with more than 1.5m off its actual location -4pts

Please submit your codes, calibration parameters, PenguinPi model (if you have made any changes to penguinpi.sdf), the best SLAM map you have generated for the testing arena, and a short readme on how to run your SLAM codes. During marking, we will run your SLAM codes both with your customoised calibration parameters and with a standard calibration parameter set. Whichever one yields better performance will be used as your mark. You SLAM map generated on the testing arena will help us identify if there is any influence of environment factors that needs to be adjusted to.

## Getting-started
### Launch the cardboard arena world (week 3)
1. If you are a git user, simply run ```git pull``` to pull the new files into your repository. **Make sure you have saved a copy of your development before pulling.** Extract [aruco_tags.zip](models/aruco_tags.zip), [Coke.zip](models/Coke.zip) and [sheep.zip](models/sheep.zip) into ```catkin_ws/src/penguinpi_gazebo/models```. Once you extracted the files, you will have three folders inside the models folder, namely "aruco_tags", "Coke", and "sheep". Inside each of these folders, there should be a "model.sdf" file (e.g., ```catkin_ws/src/penguinpi_gazebo/models/Coke/model.sdf```), which allows Gazebo to spawn that model when the world is launched. If there is an existing folder for any of these models, you can replace them.
2. Copy [penguinpi_arena.world](models/penguinpi_arena.world) into ```catkin_ws/src/penguinpi_gazebo/worlds```.
3. Copy [penguinpi_arena.launch](models/penguinpi_arena.launch) into ```catkin_ws/src/penguinpi_gazebo/launch```.
4. Copy [penguinpi.sdf](models/penguinpi.sdf) and replace the original model file in ```catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf```. This new PenguinPi model has increased wheel frictions so it's less likely for the robot to over-turn or drift after it stops.
5. Run ```source ~/catkin_ws/devel/setup.bash``` in terminal.
6. Run ```roslaunch penguinpi_gazebo penguinpi_arena.launch``` in terminal.
7. You should see a cardboard arena with ARUCO markers on its walls in the Gazebo simulator. Inside there should be a PenguinPi robot, two sheep figurines and two coke cans. This will be your testing arena. You can command the robot to move inside the arena and get its camera feed using the keyboard teleoperation you developed in Week 2. Note that the marking environment will have a different layout than the testing environment. You can also vary the environment yourself by editing ```penguinpi_arena.world``` during your development.

![PenguinPi Arena](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/PenguinPiArena.png?raw=true "The Cardboad Arena with PenguinPi robot and figurines in there")

8. AWS remote desktop works in the same way as a local VM.

9. Note: If you are using ROS development studio, Gazebo webservice visulization has some texture issue and doesn't load some objects or colors properly. However, in the "Graphical Tools", camera feed of the robot looks the same as inside local VM, only the Gazebo visualisation has trouble. You should still be able to do mapping and localisation using the camera feed. 

    You can also use gzclient that opens Gazebo inside the "Graphical Tools", which loads the objects and colors but is more laggy. 

    To launch the arena inside RDS, open "Tools" -> "Shell" and inside the terminal opened type:

    ```
    sudo apt update
    sudo apt install python-flask python-gevent
    python3 -m pip install flask gevent pyyaml numpy requests opencv-python pynput opencv-contrib-python==4.1.2.30 matplotlib
    source ~/catkin_ws/devel/setup.bash
    roslaunch penguinpi_gazebo penguinpi_arena.launch
    ```

    You should see Rviz opened inside the "Graphical Tools" at the point. To launch gzclient, open another terminal and type ```/usr/bin/gzclient-9.0.0```. You should now see Gazebo opened inside the "Graphical Tools" in addition to Rviz. It may take some time to start.

![PenguinPi Arena RDS Pale](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/PaleRDSarena.png?raw=true "The arena in ROS Development Studio's Gazebo webservice appears pale")

### Motion model (week 3)
Using the drive signals, you can estimate the resulting location of the robot. **Please fill in the code segment in [Robot.py](slam/Robot.py) ([line 27](slam/Robot.py#L27)) to complete this motion model.** As shown in [Measurements.py](slam/Measurements.py), this motion model will be used by the SLAM function.

**You may improve this estimation by calibrating the wheels**: [default wheel calibration settings](calibration/wheel_calibration) have been provided. However, this calibration may not apply to your environment. **To re-calibrate the wheels, please fill in the code segment in [wheel_calibration.py](calibration/wheel_calibration.py) ([line 87](calibration/wheel_calibration.py#L87)) to complete the calculation of the baseline parameter.** This script will drive the robot straight at different speeds, and then spin it at different speeds (feel free to change the speed ranges and steps). Note: this script needs [penguinPiC.py](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week01-02/penguinPiC.py) from Week 2 inside the same directory when running.

You can drag and move the robot inside Gazebo. The ground grids are 1m by 1m. By clicking on World -> PenguinPi -> pose you can view exact coordinates of the robot for calibration.

![Gazebo Pose Info](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/GazeboPoseInfo.png?raw=true "Gazebo Pose Info")

### ARUCO marker detection (week 4)
Other than drive signals, the SLAM function also makes use of the ARUCO makers to improve its mapping and localisation accuracy.

To visualize ARUCO marker detection, replace the keyboard teleoperation and GUI parts of [keyboardControlARtestStarter.py](keyboardControlARtestStarter.py) with your own codes (or copy [lines 92 to 106](keyboardControlARtestStarter.py#L92) into relevant part of your own keyboard teleoperation function). After the cardboard arena is launched, run ```python3 keyboardControlARtestStarter.py``` in a new terminal window (note: this script calls [penguinPiC.py](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week01-02/penguinPiC.py) from Week 2 when running). Don't forget to [install the cv2.aruco module](#ARUCO-markers). This script demonstrates ARUCO marker detection in the camera feed. Try driving the robot around and view the ARUCO marker detection annotations in its camera feed.

To visualize how noise influences ARUCO marker detection, uncomment [lines 93 to 95](keyboardControlARtestStarter.py#L93) in keyboardControlARtestStarter.py and re-run it. You should now see noises added to PenguinPi's camera feed. Try driving the robot around again and view how the ARUCO marker detection annotations change when there are noises. There will NOT be noise added during marking, this is just for your to visualize the impact of noise, which may occur when you are using a physcial robot. 

As shown in [aruco_detector.py](slam/aruco_detector.py) and [Measurements.py](slam/Measurements.py), the ARUCO marker information will be used by the SLAM function.

**You may improve this estimation by calibrating the camera**: [default camera calibration settings](calibration/camera_calibration) have been provided. However, this calibration may not apply to your environment. If you would like to re-calibrate the camera, launch [penguinpi_calibration.world](calibration/penguinpi_calibration.world) with [penguinpi_calibration.launch](calibration/penguinpi_calibration.launch) by typing ```roslaunch penguinpi_gazebo penguinpi_calibration.launch``` in the terminal (make sure you have these two files saved in the world and launch folders). Inside Gazebo you can also drag and move the robot around so that its camera faces the ARUCO checkerboard. Run [camera_calibration.py](calibration/camera_calibration.py) in a new terminal with python3 and take 20 pictures of the ARUCO checkerboard from different angles / distances to calibrate the camera.

### SLAM (week 5)
The script [manualSLAM.py](manualSLAM.py) makes use of [keyboardControlARtestStarter.py](keyboardControlARtestStarter.py), the camera / wheels [calibration info](calibration/), and the [SLAM](slam/) components to produce a map of where the ARUCO markers are as "slam.txt", which contains the list of ARUCO makers, their x-y coordinates, and the covariances. 

**Please fill in the code segment in [Slam.py](slam/Slam.py) ([line 45](slam/Slam.py#L45) and [line 71](slam/Slam.py#L71)) to complete Extended Kalman filter and finish you implementation of SLAM. Feel free to make changes to the current implementation of SLAM as it has a lot of space for improvements.** 

The list of ARUCO markers and their actual x-y coordinates in the testing arena are provided in [PenguinPiArenaTruePose.csv](PenguinPiArenaTruePose.csv), which you can compare your "slam.txt" outputs with to see how accurate your SLAM is.

## Acknowledgement
The cardboard arena is inspired by https://github.com/rfzeg/cardboard_arena

More Gazebo models: https://github.com/osrf/gazebo_models
