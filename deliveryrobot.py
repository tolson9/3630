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
	def lost(self):
		print("LOST")
		#localize
		if(self.robotPos and self.robotPos.confidence):
			#robot has localized
			print("localized")
			return "GOTOHOME"
		else:
			#robot is not localized
			self.robot.turn_in_place(degrees(15)).wait_for_completed()
			updateEasy(self.robot, self.particlefilter)
			time.sleep(.75)
			return "LOST"

	def gotohome(self):
		current_pos = self.robotPos
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


	def survey(self):
		#robot is at home position
		#check if the robot needs to survey
		if(self.grid.dronemarker):
			if(self.grid.inspectmarker):
				if(self.grid.planemarker):
					if(self.grid.ordermarker):
						return "SORTCUBES"
		#not all important markers found
		#drive to each marker and figure map the markers

		#default survey loop
		path = [self.robotPos]
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

		go_to_node(self.robotPos, path[1])
		identifymarker((1,9))

		go_to_node(self.robotPos, path[2])
		go_to_node(self.robotPos, path[3])
		identifymarker((13,18))

		#ect, ect, go to all positions and identify all the markers
		return "GOTOHOME"

	def sortcubes(self):
		#check each corner

		#count number of cubes moved
		count = 0

		#coner1
		turn_to_face(self.robotPos, (1,18))
		found = detect_cube()
		if found:
			count += 1

			path = get_path_between(self.robotPos, self.cmap.get_goal())
			follow_path(path)
			pick_up_cube()
			path = get_path_between(self.robotPos, self.grid.dronemarker)
			follow_path(path)
			drop_cube()
			found = False

		#coner2
		turn_to_face(self.robotPos, (23,18))
		found = detect_cube()
		if found:
			count += 1

			path = get_path_between(self.robotPos, self.cmap.get_goal())
			follow_path(path)
			pick_up_cube()
			path = get_path_between(self.robotPos, self.grid.placemarker)
			follow_path(path)
			drop_cube()
			found = False

		#coner3
		turn_to_face(self.robotPos, (1,1))
		found = detect_cube()
		if found:
			count += 1

			path = get_path_between(self.robotPos, self.cmap.get_goal())
			follow_path(path)
			pick_up_cube()
			path = get_path_between(self.robotPos, self.grid.inspectmarker)
			follow_path(path)
			drop_cube()
			found = False

		#coner4
		turn_to_face(self.robotPos, (23,1))
		found = detect_cube()
		if found:
			count += 1

			path = get_path_between(self.robotPos, self.cmap.get_goal())
			follow_path(path)
			pick_up_cube()
			path = get_path_between(self.robotPos, self.grid.ordermarker)
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

	def turn_to_face(self, current_pos, goal_pos):
		current_angle = current_pos[2]

		relative_x = goal_pos[0] - current_pos[0]
		relative_y = goal_pos[1] - current_pos[1]

		if(current_angle > 180) :
			current_angle -= 360
			#angle between -180 and 180

		goal_angle = math.degrees(math.atan2(relative_y, relative_x))
		change_in_angle = goal_angle - current_angleu

		#make robot turn that amount. probably with turn in place
		self.robot.turn_in_place(degrees(change_in_angle)).wait_for_completed()
		updateEasy(self.robot, self.particlefilter)

	def go_to_node(self, current_pos, goal_pos):
		turn_to_face(current_pos, goal_pos)
		relative_x = goal_pos[0] - current_pos[0]
		relative_y = goal_pos[1] - current_pos[1]
		dist = math.sqrt((relative_x)**2 + (relative_y)**2)
		#drive straight amount = dist
		action = self.robot.drive_straight(distance_inches(dist), speed_mmps(100)).wait_for_completed()
		updateEasy(self.robot, self.particlefilter)

	def get_path_between(self, start_pos, goal_pos):
		cmap.add_goal(goal_pos)
		RRT(cmap, start_pos)
		path = cmap.get_smooth_path()
		return path

	def follow_path(self, path):
		i = 1
		while(robotPos.confidence and i < len(path)):
			go_to_node(robotPos, path[i])
			self.robotPos.update(updateEasy(self.robot, self.particlefilter))
			i += 1

	def identifymarker(self, location):
		clf = self.imgClassifier
		latest_image = self.robot.world.latest_image
		new_image = latest_image.raw_image
		new_image.save("./temp/temp_.bmp")
		test_raw = []
		test_raw.append(io.imread('./temp/temp_.bmp'))
		test_data = clf.extract_image_features(test_raw)
		label = clf.predict_labels(test_data)
		label = str(label[0]).upper()

		self.grid.add_identified_marker(label, location)

	def detectcube(self):
		update_cmap, goal_center = detect_cube_and_update_cmap(self.robot, {}, self.robotPos)
		if(update_cmap):
			return True
		else :
			self.cmap.set_goal(self.robotPos)
			return False

async def run(robot: cozmo.robot.Robot):
	delrob = DeliveryRobot()
	delrob.robot = robot
	state = "LOST"
	#statemachine
	while True:
		if state == "LOST":
			state = delrob.lost()
		elif state == "GOTOHOME":
			state = delrob.gotohome()
		elif state == "SURVEY":
			state = delrob.survey()
		elif state == "SORTCUBES":
			state = delrob.sortcubes()
		elif state == "SUCCESS":
			print("SUCCESS!")
			return "GOTOHOME"
		else:
			return "LOST"

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