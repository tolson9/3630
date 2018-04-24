import math
import random

import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image

from cmap import *
from rrt import *

from particle import *
from particlefilter import *

from position import *

from grid import *

from imageclassifier import *

from markers import detect, annotator


class DeliveryRobot:

	def __init__(self):
		#setup
		self.robot = None
		self.robotPos = None

		self.grid = CozGrid("map_arena.json")
		self.cmap = CozMap("gridwithbox.json", node_generator)

		self.particlefilter = ParticleFilter(self.grid)
		self.imgClassifier = ImageClassifier()
		self.imgClassifier.classifer = joblib.load('classifier.pk1')

#lost state
async def lost(delrob):
	print("LOST")
	#localize
	if(delrob.robotPos and delrob.robotPos.confidence):
		#robot has localized
		print("localized")
		return "GOTOHOME"
	else:
		#robot is not localized
		action = delrob.robot.turn_in_place(degrees(15))
		time.sleep(0.5)


		updateEasy(delrob.robot, delrob.particlefilter)
		print("------------------------------------------------------------------------------------------")

		time.sleep(.75)
		return "LOST"

async def gotohome(delrob):
	current_pos = delrob.robotPos
	home_pos = Position(8,8,0,1)

	if(robotPos and robotPos.confidence):
		#plan a path to home
		path = get_path_between(current_pos, home_pos)

		#use path
		follow_path(path)

		if(robotPos.confidence):
			#reached home
			return "SURVEY"
		else:
			return "LOST"

	else:
		return "LOST"


async def survey(delrob):
	#robot is at home position
	#check if the robot needs to survey
	if(delrob.grid.dronemarker):
		if(delrob.grid.inspectmarker):
			if(delrob.grid.planemarker):
				if(delrob.grid.ordermarker):
					return "SORTCUBES"
	#not all important markers found
	#drive to each marker and figure map the markers

	#default survey loop
	path = [delrob.robotPos]
	path.append((7,8))

	path.append((13,14))
	path.append((13,13))

	path.append((18,11))
	path.append((17,10))

	path.append((17,9))
	path.append((18,9))

	path.append((17,9))
	path.append((17,8))

	path.append((11,8))
	path.append((11,7))

	path.append((8,8))

	go_to_node(delrob.robotPos, path[1])
	identifymarker((1,9))

	go_to_node(delrob.robotPos, path[2])
	go_to_node(delrob.robotPos, path[3])
	identifymarker((13,18))

	#ect, ect, go to all positions and identify all the markers
	return "GOTOHOME"

async def sortcubes(delrob):
	#check each corner

	#count number of cubes moved
	count = 0

	#coner1
	turn_to_face(delrob.robotPos, (1,18))
	found = detect_cube()
	if found:
		count += 1

		path = get_path_between(delrob.robotPos, delrob.cmap.get_goal())
		follow_path(path)
		pick_up_cube()
		path = get_path_between(delrob.robotPos, delrob.grid.dronemarker)
		follow_path(path)
		drop_cube()
		found = False

	#coner2
	turn_to_face(delrob.robotPos, (23,18))
	found = detect_cube()
	if found:
		count += 1

		path = get_path_between(delrob.robotPos, delrob.cmap.get_goal())
		follow_path(path)
		pick_up_cube()
		path = get_path_between(delrob.robotPos, delrob.grid.placemarker)
		follow_path(path)
		drop_cube()
		found = False

	#coner3
	turn_to_face(delrob.robotPos, (1,1))
	found = detect_cube()
	if found:
		count += 1

		path = get_path_between(delrob.robotPos, delrob.cmap.get_goal())
		follow_path(path)
		pick_up_cube()
		path = get_path_between(delrob.robotPos, delrob.grid.inspectmarker)
		follow_path(path)
		drop_cube()
		found = False

	#coner4
	turn_to_face(delrob.robotPos, (23,1))
	found = detect_cube()
	if found:
		count += 1

		path = get_path_between(delrob.robotPos, delrob.cmap.get_goal())
		follow_path(path)
		pick_up_cube()
		path = get_path_between(delrob.robotPos, delrob.grid.ordermarker)
		follow_path(path)
		drop_cube()
		found = False

	if count >= 3:
		return "SUCCESS"
	else :
		return "SORTCUBES"

#functions to move cozmo
# def relative_to_global(relative_origin, relative_pos):
# 	current_angle = relative_origin[2]
# 	#figure out what relative pos gives us and use it get relative_x/y

# 	if(current_angle > 180) :
# 		current_angle -= 360
# 		#angle between -180 and 180
# 	goal_angle = math.degrees(math.atan2(relative_y, relative_x))

# 	change_in_angle = goal_angle - current_angle

# 	dist = math.sqrt((relative_x)**2 + (relative_y)**2)

# 	new_x = relative_origin[0] + rel_dist*math.cos(math.radians(final_angle))

# 	new_y = current_y + rel_dist*math.sin(math.radians(final_angle))

async def turn_to_face(delrob, current_pos, goal_pos):
	current_angle = current_pos[2]

	relative_x = goal_pos[0] - current_pos[0]
	relative_y = goal_pos[1] - current_pos[1]

	if(current_angle > 180) :
		current_angle -= 360
		#angle between -180 and 180

	goal_angle = math.degrees(math.atan2(relative_y, relative_x))
	change_in_angle = goal_angle - current_angleu

	#make robot turn that amount. probably with turn in place
	delrob.robot.turn_in_place(degrees(change_in_angle)).wait_for_completed()
	updateEasy(delrob.robot, delrob.particlefilter)

async def go_to_node(delrob, current_pos, goal_pos):
	turn_to_face(current_pos, goal_pos)
	relative_x = goal_pos[0] - current_pos[0]
	relative_y = goal_pos[1] - current_pos[1]
	dist = math.sqrt((relative_x)**2 + (relative_y)**2)
	#drive straight amount = dist
	action = delrob.robot.drive_straight(distance_inches(dist), speed_mmps(100)).wait_for_completed()
	updateEasy(delrob.robot, delrob.particlefilter)

def get_path_between(delrob, start_pos, goal_pos):
	cmap.add_goal(goal_pos)
	RRT(cmap, start_pos)
	path = cmap.get_smooth_path()
	return path

async def follow_path(delrob, path):
	i = 1
	while(robotPos.confidence and i < len(path)):
		go_to_node(robotPos, path[i])
		delrob.robotPos.update(updateEasy(delrob.robot, delrob.particlefilter))
		i += 1

async def identifymarker(delrob, location):
	clf = delrob.imgClassifier
	latest_image = delrob.robot.world.latest_image
	new_image = latest_image.raw_image
	new_image.save("./temp/temp_.bmp")
	test_raw = []
	test_raw.append(io.imread('./temp/temp_.bmp'))
	test_data = clf.extract_image_features(test_raw)
	label = clf.predict_labels(test_data)
	label = str(label[0]).upper()

	delrob.grid.add_identified_marker(label, location)

async def detectcube(delrob):
	update_cmap, goal_center = detect_cube_and_update_cmap(delrob.robot, {}, delrob.robotPos)
	if(update_cmap):
		return True
	else :
		delrob.cmap.set_goal(delrob.robotPos)
		return False

async def updateEasy(robot, filt):
    odom = compute_odometry(robot.pose)
    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)

    markers, camera_image = await marker_processing(robot=robot, camera_settings=camera_settings)

    (m_x,m_y,m_h,confidence) = filt.update(odom, markers)
    return (m_x, m_y, m_h, m_confident)

async def run(robot: cozmo.robot.Robot):
	delrob = DeliveryRobot()
	delrob.robot = robot
	state = "LOST"

	updateEasy(robot, delrob.particlefilter)

	#statemachine
	while True:
		if state == "LOST":
			state = await lost(delrob)
		elif state == "GOTOHOME":
			state = await gotohome(delrob)
		elif state == "SURVEY":
			state = await survey(delrob)
		elif state == "SORTCUBES":
			state = await sortcubes(delrob)
		elif state == "SUCCESS":
			print("SUCCESS!")
			state = "GOTOHOME"
		else:
			state = "LOST"

class CozmoThread(threading.Thread):
    
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()