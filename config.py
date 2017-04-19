# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 16:15:43 2017

@author: ozer

Bunch of global values for other stuff to read from.
"""
###############################################################################
# Agents
###############################################################################
#how many mobile agents will be simulated
NUMBER_OF_AGENTS = 2
#agent shape radius, in meters
AGENT_SHAPE_RADIUS = 1.5
#agent head shape radius
AGENT_HEAD_RADIUS = 0.5
#max forward speed and angular speeds
AGENT_MAX_V = 1.5 #m/s 1.5 normal
AGENT_MAX_W = 30. #degrees 15 normal


###############################################################################
# Simulation
###############################################################################
#simulation updates per second
UPDATE_FPS = 60.
#display size in pixels
WINDOW_SIZE = 600
#display size in sim. meters
WINDOW_METERS = 150. #1k final
#display size in inches of screen
WINDOW_INCHES = 5.
#window dpi
WINDOW_DPI = WINDOW_SIZE/WINDOW_INCHES
#size of the depthmap in pixels
DEPTHMAP_SIZE = WINDOW_SIZE
#depthmap octave value, defines the 'islandy'ness of the generated maps. Higher=larger islands
DEPTHMAP_OCTAVES = 12
#should the simulation start the control processes or will they be started manually?
START_CONTROLS = False

#noise for gps and sonar measurements
GPS_POS_NOISE = 0.1
SONAR_NOISE = 0.1

###############################################################################
# Sensor periods in seconds. Simulates the time to reset of sensors
###############################################################################
GPS_FREQ = 1.
NRG_FREQ = 1.
SNR_FREQ = 1.
NET_FREQ = 1.

###############################################################################
# Sensor polling rates.
# These are used by the agent to determine when to ask for a new value
# Made different specifically
###############################################################################
GPS_POLL = 0.5
NRG_POLL = 0.5
SNR_POLL = 0.5
NET_POLL = 0.5

###############################################################################
# Windows
###############################################################################
#display size for agent control windows
CONTROL_WIN_SIZE = 600
CONTROL_PPM = CONTROL_WIN_SIZE / WINDOW_METERS*1.
#display pixels per meter
PPM = WINDOW_SIZE / WINDOW_METERS*1.


###############################################################################
# UDP multicast addresses and ports.
###############################################################################
#use all interfaces on the port
HOST_IP = "0.0.0.0"
#ports where everything will dump their data into
SENDER_PORT = 1501
#multicast params
MCAST_ADDR = "224.168.2.9"
MCAST_PORT = 1600
TTL = 31# valid value are 1-255, <32 is local network


###############################################################################
# Comm. commands should be changed to NMEA sentences when working
# the real agents. Otherwise, the Body's of agents should have a handler funct.
# of the same name.
###############################################################################
GET_GPS = 'get_gps'
GET_ENERGY = 'get_nrg'
GET_SONAR = 'get_snr'
GET_NETWORK = 'get_net'

#SET_TARGET = 'set_tgt'

###############################################################################
# colormap for the displays
###############################################################################
COLORMAP_FILE = 'colormap.matrix'
DEPTHMAP_FILE = 'depthmap.png'
DEPTHMAP_BASE = 1
DEPTHMAP_SMOOTHING = 1

#depth range of the seabed. negative depth = land above water
DEPTHMAP_MINDEPTH = -10
DEPTHMAP_MAXDEPTH = 50

#distance limit to record measurements for agents. Not moving or moving too slowly
#causes instabilities for GP
# in meters
DISTANCE_THRESHOLD = 1.

#a directory to save agent traces
TRACE_DIR = 'traces/'

#distance threshold to consider a target reached
TARGET_DISTANCE_THRESHOLD = 2.
#angle threshold to consider a target 'in front'. In degrees
TARGET_ANGLE_THRESHOLD = 0.1

#number of measurements expected to start using gp
MEASUREMENT_COUNT_THRESHOLD = 5

#min. amount of std. to consider a point 'unexplored'
#0.02 is about the same as a point with a measurement right next to it
MIN_STD = 0.15

#min number of points to expect to consider choosing one
MIN_UNEXPLORED = 5

#default values of smallest search donut
DEFAULT_START_RANGE = 10
DEFAULT_END_RANGE = 30
DEFAULT_CIRCLE_COUNT = 120

#incerement in range when no points is found
SEARCH_INCREMENT = 20

#a flag to allow agents to paint un/explored areas in their control windows
PAINT_EXPLORED = True
PAINT_UNEXPLORED = False

#should the agent ignore broadcasts that are far away?
SIMULATE_NETWORK_BREAKAGE = True
#range to start ignoring messages
NETWORK_RANGE = 50