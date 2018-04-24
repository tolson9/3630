from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
np.random.seed(RANDOM_SEED)
from itertools import product
import math
import numpy as np
import time
import asyncio
from PIL import Image

import cozmo

from markers import detect, annotator

class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        Map_filename = "map_arena.json"
        self.grid = CozGrid(Map_filename)

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False


def compute_odometry(curr_pose, cvt_inch=False):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees
    
    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


async def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera. 
    Since this is an async function, it must be called using await, for example:

        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh) 
          (as expected by the particle filter's measurement update)
        - a PIL Image of what Cozmo's camera sees with marker annotations
    '''

    global grid

    # Wait for the latest image from Cozmo
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # Convert the image to grayscale
    image = np.array(image_event.image)
    image = color.rgb2gray(image)
    
    # Detect the markers
    markers, diag = detect.detect_markers(image, camera_settings, include_diagnostics=True)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x,y,h in marker_list]

    # Annotate the camera image with the markers
    if not show_diagnostic_image:
        annotated_image = image_event.image.resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(annotated_image, markers, scale=2)
    else:
        diag_image = color.gray2rgb(diag['filtered_image'])
        diag_image = Image.fromarray(np.uint8(diag_image * 255)).resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(diag_image, markers, scale=2)
        annotated_image = diag_image

    return marker_list, annotated_image

def diff_heading_deg(heading1, heading2):
    dh = heading1 - heading2
    while dh > 180:
        dh -= 360
    while dh <= -180:
        dh += 360
    return dh

def grid_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    for p in particles:
        #motion_particles.append(Particle(x= add_gaussian_noise(p.x + odom[0], 0.2), y=add_gaussian_noise(p.y + odom[1], 2), heading = add_gaussian_noise(p.h + odom[2], 0.5)))
        (x,y) = rotate_point(odom[0], odom[1], diff_heading_deg(p.h, odom[2]))
        xVal = add_gaussian_noise(p.x + x,0.01)
        yVal = add_gaussian_noise(p.y + y,0.01)
        hVal = add_gaussian_noise(p.h + odom[2], 1)

        motion_particles.append(Particle(x=xVal, y=yVal, heading=hVal))

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    if len(measured_marker_list) == 0:
        #print("Cant see marker")
        measured_particles = []
        for p in particles :
            if not grid.is_in(p.x, p.y) :
                if p.x > 0 :
                    p.x -= 0.25
                else :
                    p.x += 0.25
                if p.y > 0 :
                    p.y -= 0.25
                else :
                    p.y += 0.25


                #(x,y) = grid.random_free_place()
                #measured_particles.append(Particle(x,y,heading=None))
                measured_particles.append(p)

            else :
                measured_particles.append(p)
        return measured_particles
    # Update Weights
    probs =[]
    for p in particles:
        count = 0
        probList = []
        bigCounter = 0
        littleCounter = len(measured_marker_list)
        
        for p_marker in p.read_markers(grid):
            bigCounter += 1
            for m_marker in measured_marker_list:
                probList.append(getProb(p_marker, m_marker))
        prob = 1
        if not probList:
                prob*= 0.1
  
        selectedProb = []

        for i in range(0, min(bigCounter, littleCounter)) :
            prob *= max(probList)
            selectedProb.append(max(probList))
            maxIndex = probList.index(max(probList))
            remNums = math.floor(maxIndex/littleCounter)
            if probList :
                for j in range(0, littleCounter):
                    if remNums + j < len(probList):
                        probList.remove(probList[remNums + j])

        prob *= 0.1**(abs(bigCounter - littleCounter))

        #print("Selected Probs")
        #print(selectedProb)
        probs.append(prob)
                #print(max(probList))

    #Resample
    #print(probs)
    for i in range(0, PARTICLE_COUNT) :
        if probs[i] < 0.05 :
          probs[i] = 0

    sum = np.sum(probs)
    if sum == 0 :
        return particles

    normalized = np.array(probs)
    normalized /= sum
    serted = sorted(normalized, reverse=True);
    #print(sum)

    measured_particles = []

    sampled = np.random.choice(particles, PARTICLE_COUNT, p = normalized)

    for p in sampled:
        if not grid.is_in(p.x, p.y) :
            (x,y) = grid.random_free_place()
            measured_particles.append(Particle(x=x,y=y, heading=None))
        else :
            measured_particles.append(p)

    for i in range(math.floor(0.975*PARTICLE_COUNT), PARTICLE_COUNT):
        (x,y) = grid.random_free_place()
        measured_particles[i] = Particle(x=x,y=y, heading=None)

    return measured_particles

#-------------------------------------------------------------------------------
def getProb(a, b) :
    dist = grid_distance(a[0], a[1], b[0], b[1])
    angle_dist = diff_heading_deg(a[2], b[2]);
    #prob = math.exp(-0.5*((dist**2)/(.02**2) + (angle_dist**2)/(2**2)))
    prob = math.exp(-0.25*((dist**2)/(2**2) + (angle_dist**2)/(10**2)))
    return prob


#setup for particle filter, move to delivery function or init
if __name__ == "__main__":
    Map_filename = "map_arena.json"
    grid = CozGrid(Map_filename)
    #gui = GUIWindow(grid, show_camera=True)

    particles = Particle.create_random(PARTICLE_COUNT, grid)
    particlefilter = ParticleFilter(particles, grid)