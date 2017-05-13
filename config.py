# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 16:15:43 2017

@author: ozer

Bunch of global values for other stuff to read from.
"""
#is the agent wokring in sim or on physical?
#changes the communication stuffs mostly
#from udp multicasts to tcp connections
#if true, runs in sim.
SIMULATION = True
USE_TAHIROVIC = True

###############################################################################
# FILES
###############################################################################
import time
#suffix added to agent-generated stuff
SUFFIX = 'physical'+'__'+str(time.time())
#colormap to color the depthmap
COLORMAP_FILE = 'colormap.matrix'
#visual of the depthmap
DEPTHMAP_FILE = 'depthmap.png'
#a directory to save agent traces
TRACE_DIR = 'traces/'

###############################################################################
# SIMULATION
###############################################################################
#how many mobile agents will be simulated
NUMBER_OF_AGENTS = 2
#starting positions of agents in meters.
INIT_POS = [[0,0],[0,5]]
#starting headings of agents in degrees. ccw positive from x axis
INIT_HEADING = [0.,0.]
#max forward speed and angular speeds
AGENT_MAX_V = 5 #m/s 1.5 normal
AGENT_MAX_W = 40. #degrees 15 normal
#simulation updates per second
UPDATE_FPS = 60.
#simulation area size. Middle point is 0,0, this value is the length of one side
#of the square area
WINDOW_METERS = 100. #1k final
#should the agent ignore broadcasts that are far away?
SIMULATE_NETWORK_BREAKAGE = False
#range to start ignoring messages in meters
NETWORK_RANGE = 10
#for simple rectangle polygon bounds for the agents to stay inside
#in meters
SAFETY_RECT = 1


###############################################################################
# TAHIROVIC
###############################################################################
# n x n grid of sampled points.
T_DENSITY = 80
# use variance to weight the centroid for tahirovic?
T_WEIGHTED = True


###############################################################################
# DEPTHMAP GENERATION
###############################################################################
#depthmap octave value, defines the 'islandy'ness of the generated maps. Higher=larger islands
DEPTHMAP_OCTAVES = 12
DEPTHMAP_BASE = 1
#gaussian blur smootness
DEPTHMAP_SMOOTHING = 10

#depth range of the seabed. negative depth = land above water
DEPTHMAP_MINDEPTH = -10
DEPTHMAP_MAXDEPTH = 50


###############################################################################
# SIMULATED SENSORS
###############################################################################
# Sensor periods in seconds. Simulates the time to reset of sensors
GPS_FREQ = 1.
NRG_FREQ = 1.
SNR_FREQ = 1.
NET_FREQ = 1.
#noise for gps and sonar measurements in meters
GPS_POS_NOISE = 0.1
SONAR_NOISE = 0.1
# Sensor polling rates.
# These are used by the agent to determine when to ask for a new value
if SIMULATION:
    GPS_POLL = 0.5
    NRG_POLL = 0.5
    SNR_POLL = 0.5
    NET_POLL = 0.5
else:
    GPS_POLL = 0.1
    NRG_POLL = 0.1
    SNR_POLL = 0.1
    NET_POLL = 0.1


###############################################################################
# AGENT BEHAVIOUR
###############################################################################
#discount points that will take longer to reach? this is the divider in utility func.
CARE_ABOUT_TTR = False
#velocity obstacle agent disc radius assumptions. Keep in mind that there is a delay
#in communications, so the 'other' radius should probably be larger than self radius
#in meters.
VO_SELF_R = 2.
VO_OTHER_R = 5.


#number of measurements expected to start using gp
MEASUREMENT_COUNT_THRESHOLD = 5
#min. amount of std.dev. to consider a point 'unexplored'
MIN_STD = 0.13
#min number of points to expect to consider choosing one
MIN_UNEXPLORED = 1
#default values of smallest search donut. in meters
DEFAULT_START_RANGE = 10
DEFAULT_END_RANGE = 30
DEFAULT_CIRCLE_COUNT = 120
#incerement in range when no points is found
SEARCH_INCREMENT = 20
#time to wait between broadcasting missing values in seconds
MISSING_INTERVAL = 5
#dont flood udp buffers with the same mment, this is for 'real-time' bcasting
#of x,y,vx,vy,d. in seconds
MMENT_INTERVAL = 0.5
#allowed proximity of agent targets to each other when searching for
#the next point to go. in meters.
TARGET_PROXIMITY_LIMIT = 5


#distance limit to record measurements for agents. Not moving or moving too slowly
#causes instabilities for GP.
if SIMULATION:
    #in meters
    DISTANCE_THRESHOLD = 0.3
else:
    #in GPS minutes
    DISTANCE_THRESHOLD = 1e-7

#distance threshold to consider a target reached
if SIMULATION:
    #in meters
    TARGET_DISTANCE_THRESHOLD = 2.
else:
    #in meters
    TARGET_DISTANCE_THRESHOLD = 2.

#angle threshold to consider a target 'in front'. In degrees.
#only used in simulations
TARGET_ANGLE_THRESHOLD = 0.1


###############################################################################
# VISUALS
###############################################################################
#display size in pixels
WINDOW_SIZE = 600
#agent shape radius, in meters
AGENT_SHAPE_RADIUS = 1.5
#agent head shape radius
AGENT_HEAD_RADIUS = 0.5
#display size for agent control windows
CONTROL_WIN_SIZE = 600
#a flag to allow agents to paint un/explored areas in their control windows
if SIMULATION:
    PAINT_EXPLORED = False
    PAINT_UNEXPLORED = False
    PAINT_TAHIROVIC = True
else:
    PAINT_EXPLORED = False
    PAINT_UNEXPLORED = False
    PAINT_TAHIROVIC = False
#draw the tahirovic centroid on the canvas?
T_DRAW_CENTROID = True


###############################################################################
# DERIVED VALUES
###############################################################################
#display size in inches of screen
WINDOW_INCHES = 5.
#window dpi
WINDOW_DPI = WINDOW_SIZE/WINDOW_INCHES
#size of the depthmap in pixels
DEPTHMAP_SIZE = WINDOW_SIZE
#pixels per meter of the control window
CONTROL_PPM = CONTROL_WIN_SIZE / WINDOW_METERS*1.
#display pixels per meter
PPM = WINDOW_SIZE / WINDOW_METERS*1.


###############################################################################
# STUFF THAT SHOULD PROBABLY BE REMOVED
###############################################################################
#should the simulation start the control processes or will they be started manually?
START_CONTROLS = False

# UDP multicast addresses and ports.
#use all interfaces on the port
HOST_IP = "0.0.0.0"
#ports where everything will dump their data into
SENDER_PORT = 1501
#multicast params
MCAST_ADDR = "224.168.2.9"
MCAST_PORT = 1600
TTL = 31# valid value are 1-255, <32 is local network

GET_GPS = 'get_gps'
GET_ENERGY = 'get_nrg'
GET_SONAR = 'get_snr'
GET_NETWORK = 'get_net'