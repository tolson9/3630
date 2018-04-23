import cozmo
import math
import sys
import time
import random

from cmap import *
from gui import *
from utils import *
from cozmo.util import *

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    ############################################################################
    x_diff = node0.coord[0] - node1.coord[0]
    y_diff = node0.coord[1] - node1.coord[1]

    distance = (x_diff)**2 + (y_diff)**2
    distance = math.sqrt(distance)

    if distance > limit:
        xchange = limit*(x_diff/distance)
        ychange = limit*(y_diff/distance)

        coord = (node0.coord[0] + xchange, node0.coord[1] + ychange)
        new_node = Node(coord= coord)
        return new_node
       
def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    while True:
        [maxX, maxY] = cmap.get_size()
        x = random.random()*maxX
        y = random.random()*maxY
        coord = (x,y)

        rand_node = Node(coord = coord)
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            return rand_node
    
    ############################################################################

def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()

        nodes = cmap.get_nodes()

        if nodes:
            nearest_node = nodes[0]
            minDist = 10**100
            for node in nodes:
                distance = (rand_node.coord[0] - node.coord[0])**2 + (rand_node.coord[1] - node.coord[1])**2
                distance = math.sqrt(distance) 
                if distance < minDist:
                    nearest_node = node
                    minDist = distance
        ########################################################################
        else:
            nearest_node = start

        new_node = step_from_to(nearest_node, rand_node)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")

async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
      # TODO: please enter your code below.
    # Description of function provided in instructionss
    print("here\n")
    #await robot.drive_straight(distance_inches(10), speed_mmps(200)).wait_for_completed()
    pos = (6, 10)
    while(True):
        cmap, goal = detect_cube_and_update_cmap(robot, {}, pos)
        while(not goal):
            #rotate until it finds goal
            action = robot.turn_in_place(degrees(15)).wait_for_completed()
            cmap, goal = detect_cube_and_update_cmap(robot, {}, pos)

        #robot should have located goal
        RRT(cmap, pos);
        path = cmap.get_smooth_path()
        numObstacles = len(cmap._obstacles)
        nodeIndex = 0;
        while(numObstacles == cmap._obstacles && nodeIndex < len(path) - 1):
            cmap, goal = detect_cube_and_update_cmap(robot, {}, pos)
            #go to next node
            current_pos = path[i];
            current_angle = (robot.pose)[2]
            next_node = path[nodeIndex + 1]

            next_angle = (next_node[1] - current_pos[1]) / (next_node[0] - current_pos[0])
            next_angle = math.arctan2(next_angle)

            await robot.turn_in_place(current_angle - next_angle)
            time.sleep(.5)
            dist = get_dist(current_pos, next_node)
            await robot.drive_straight(distance_inches(dist), speed_mmps(200)).wait_for_completed()
            pos = next_nodes
        if numObstacles = cmap._obstacles :
            #successfully reached goal
            robot.speak("Success")
            time.sleep(3)

async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    #updates the map with observed cubes and sets the goal if it is found
    #marked can be initialized to {}
    
    global cmap
    cube_padding = 60.
    cozmo_padding = 100.
    goal_cube_found = False
    update_cmap = False
    goal_center = None
    for obj in robot.world.visible_objects:
        if obj.object_id in marked:
            continue

        print(obj)
        update_cmap = True
        is_goal_cube = robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id

        robot_pose = robot.pose
        object_pose = obj.pose

        dx = object_pose.position.x - robot_pose.position.x
        dy = object_pose.position.y - robot_pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))

        angle = object_pose.rotation.angle_z.radians

        if is_goal_cube:
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(angle, object_pos, local_goal_pos)
            cmap.clear_goals()
            cmap.add_goal(goal_pos)
            goal_cube_found = True
            goal_center = object_pos

        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(angle, object_pos, 
                                              Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(angle, object_pos, 
                                              Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(angle, object_pos, 
                                              Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(angle, object_pos, 
                                              Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)

        marked[obj.object_id] = obj

    return update_cmap, goal_center

def get_global_node(angle,object_pos, Node):
    

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)    
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
