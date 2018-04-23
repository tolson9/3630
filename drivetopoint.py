import math
import random

def drivetopoint(current_pos, goal_pos):

	#relative_x = goal_pos.x - current_pos.x
	#relative_y = goal_pos.y - current_pos.y

	#current_angle = current_pos.angle

	current_x = current_pos[0]
	current_y = current_pos[1]
	current_angle = current_pos[2]

	goal_x = goal_pos[0]
	goal_y = goal_pos[1]


	relative_x = goal_x - current_x
	relative_y = goal_y - current_y


	if(current_angle > 180) :
		current_angle -= 360
		#angle between -180 and 180

	goal_angle = math.degrees(math.atan2(relative_y, relative_x))

	change_in_angle = goal_angle - current_angle

	dist = math.sqrt((relative_x)**2 + (relative_y)**2)

	#checks
	final_angle = current_angle + change_in_angle

	final_x = current_x + dist*math.cos(math.radians(final_angle))

	final_y = current_y + dist*math.sin(math.radians(final_angle))

	return (final_x, final_y)


def main():
	total = 0;
	errs = []
	for i in range(0,10):
		x = random.randint(0, 10)
		y = random.randint(0, 10)
		angle = random.randint(0, 360)

		current_pos = (x,y,angle)

		x2 = random.randint(0, 10)
		y2 = random.randint(0, 10)

		goal_pos = (x2, y2)

		(final_x, final_y) = drivetopoint(current_pos, goal_pos)

		distance_from_target = math.sqrt((final_x - x2)**2 + (final_y - y2)**2)

		if(distance_from_target > 0.1) :
			errs.append((current_pos,goal_pos))
		
		total += distance_from_target;

	avg = total / 10;
	return (avg, errs)


