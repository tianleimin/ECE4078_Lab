import numpy as np
import os
import sys
import rospy
from gazebo_msgs.srv import GetModelState

import penguinPiC
ppi = penguinPiC.PenguinPi()

ppi.set_velocity(20, 20, 10)

# model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# pi = model_coordinates("PenguinPi", "ground_plane")

# print(pi)