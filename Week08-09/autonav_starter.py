# assumption of this basic vision-based auto nav implementation: 
# if the robot can see a marker, then that means there is a path to that marker

# implementation explanation
# the robot first spins 360 to find all visible markers and their estimated pose
# it then creates a list of markers that is reachable from current location and their estimated distance
# the robot now goes to the nearest reachable marker A
# once reached marker A, the robot spins 360 again to find all accessable markers from marker A and their estimated pose
# repeat until all markers are found or timed out

# import required modules
import time
import cv2
import numpy as np
import cv2.aruco as aruco

# don't forget to put penguinPiC.py in the same directory
import penguinPiC
ppi = penguinPiC.PenguinPi()

# initialize resulting map containing paths between markers
marker_list = []
saved_map = []
map_f = 'map.txt'
# there are 6 markers in total in the arena
total_marker_num = 6

# drive settings, feel free to change them
wheel_vel = 40
fps = 10

# camera calibration parameters (from M2: SLAM)
camera_matrix = np.loadtxt('camera_calibration/intrinsic.txt', delimiter=',')
dist_coeffs = np.loadtxt('camera_calibration/distCoeffs.txt', delimiter=',')
marker_length = 0.1

# wheel calibration parameters (from M2: SLAM)
wheels_scale = np.loadtxt('wheel_calibration/scale.txt', delimiter=',')
wheels_width = np.loadtxt('wheel_calibration/baseline.txt', delimiter=',')

# display window for visulisation
cv2.namedWindow('video', cv2.WINDOW_NORMAL);
cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);
# font display options
font = cv2.FONT_HERSHEY_SIMPLEX
location = (0, 0)
font_scale = 1
font_col = (255, 255, 255)
line_type = 2

# initial location of the robot
robot_pose = [0,0]
current_marker = 'start'

# 15 minutes time-out to prevent being stuck during auto nav
timeout = time.time() + 60*15  

start_t = time.time()
# repeat until all markers are found or until time out
while len(marker_list) < total_marker_num:
    # ------------------------------------------------------------------------------------
    # TODO: calculate the time the robot needs to spin 360 degrees
    # spin_time = ?
    # ------------------------------------------------------------------------------------

    # save all the seen markers and their estimated poses at each step
    measurements = []
    seen_ids = []
    for step in range(int(spin_time*fps)):
        # spinning and looking for markers at each step
        ppi.set_velocity(-wheel_vel, wheel_vel, 1/fps)
        ppi.set_velocity(0, 0)

        # get current frame
        curr = ppi.get_image()

        # visualise ARUCO marker detection annotations
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.minDistanceToBorder = 0
        aruco_params.adaptiveThreshWinSizeMax = 1000
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares

        # scale to 144p
        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

        # add GUI text
        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

        # visualisation
        cv2.imshow('video', resized)
        cv2.waitKey(1)

        # compute a marker's estimated pose and distance to the robot
        if ids is None:
            continue
        else:
            for i in range(len(ids)):
                idi = ids[i,0]
                # Some markers appear multiple times but should only be handled once.
                if idi in seen_ids:
                    continue
                else:
                    seen_ids.append(idi)
                # get pose estimation
                # ------------------------------------------------------------------------------------
                # TODO: this is a basic implementation of pose estimation, change it to improve your auto nav
                lm_tvecs = tvecs[ids==idi].T
                lm_bff2d = np.block([[lm_tvecs[2,:]],[-lm_tvecs[0,:]]])
                lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1,1)
                # compute Euclidean distance between the robot and the marker
                dist = np.sqrt((lm_bff2d[0][0]-robot_pose[0]) ** 2 + (lm_bff2d[1][0]-robot_pose[1]) ** 2)
                # save marker measurements and distance
                lm_measurement = [idi, dist, lm_bff2d[0][0], lm_bff2d[1][0]]
                measurements.append(lm_measurement)
                # ------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------
    # expand the map by going to the nearest marker
    # TODO: notice that the robot can get stuck always trying to reach the nearest marker, improve the search strategy to improve auto nav
    measurements = sorted(measurements, key=lambda x: x[1]) # sort seen markers by distance (closest first)

    if len(measurements) > 0:
        # add discovered markers to map
        for accessible_marker in measurements:
            if current_marker != accessible_marker[0]: # avoid adding path to self
                path = []
                path.append(current_marker)
                path.append(accessible_marker[0])
                path.append(accessible_marker[1])
                saved_map.append(path)
                if accessible_marker[0] not in [found[0] for found in marker_list]: # avoid adding repeated marker
                    marker_list.append([accessible_marker[0], accessible_marker[2], accessible_marker[3]])
                else:
                    continue
            else:
                continue

        # drive to the nearest marker by first turning towards it then driving straight
        # TODO: calculate the time the robot needs to spin towards the nearest marker
        # turn_time = ?
        ppi.set_velocity(-wheel_vel, wheel_vel, turn_time)
        ppi.set_velocity(0, 0)
        # TODO: calculate the time the robot needs to drive straight to the nearest marker
        # drive_time = ?
        ppi.set_velocity(wheel_vel, wheel_vel, drive_time)
        ppi.set_velocity(0, 0)
        # TODO: you may implement an alterative approach that combines turning and driving forward

        # update the robot's pose to location of the marker it tries to reach
        # TODO: notice that the robot may not reach the marker, improve auto nav by improving the pose estimation
        robot_pose = [measurements[0][2],measurements[0][3]]
        current_marker = measurements[0][0]

        print('current map [current marker id, accessible marker id, distance]:\n',saved_map)
        print('current marker list [id, x, y]:\n',marker_list)

    else:
        print('no markers in sight!')
    # ------------------------------------------------------------------------------------

    # time out after 15 minutes
    if time.time() > timeout:
        break

# show time spent generating the map
end_t = time.time()
map_t = (end_t - start_t) / 60
print('time spent generating the map (in minutes): ',map_t)

# save results to map.txt
# sort marker list by id before printing
marker_list = sorted(marker_list, key=lambda x: x[0])
with open(map_f,'w') as f:
    f.write('id, x, y\n')
    for markers in marker_list:
        for marker in markers:
            f.write(str(marker) + ',')
        f.write('\n')
    f.write('\ncurrent id, accessible id, distance\n')
    for routes in saved_map:
        for route in routes:
            f.write(str(route) + ',')
        f.write('\n')
print('map saved!')
