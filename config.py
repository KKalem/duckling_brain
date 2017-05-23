# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 16:15:43 2017

@author: ozer

Bunch of global values for other stuff to read from.
"""
import numpy as np
import util as u
import time

#suffix added to agent-generated stuff
SUFFIX = '__1k_hybrid_004__'#+'__'+str(time.time())


#1k_hybrid_004 -> 67k seconds

#is the agent wokring in sim or on physical?
#changes the communication stuffs mostly
#from udp multicasts to tcp connections
#if true, runs in sim.
SIMULATION = True
#wether the agent has a screen or not.
HEADLESS = False
#use the tahirvoic method?
USE_TAHIROVIC = True
#use ONLY the tahirovic method w/o GP?
#overrides the above
USE_PURE_TVIC = False

#MIN_STD = 0.13
#MIN_STD = 0.09
MIN_STD = 0.04

#bounding poly agents will stay in. ccw direction
if SIMULATION:
    import geometry as gm
    #everything is in meters
    WINDOW_METERS = 1000.
    #square side = 2a, centered at 0,0
    a = WINDOW_METERS/2.
    #safety distance to shrink
    s = 1
    POLY = [[ a-s, a-s],
            [-a+s, a-s],
            [-a+s,-a+s],
            [ a-s,-a+s]]

#    POLY = [[-3.6811352, 41.3639399],
#            [-45.5642738, -3.8447412],
#            [-23.3138564, -43.9282137],
#            [22.3639399, -28.3856427],
#            [34.4390651, 36.7295492]]
#
#    POLY = [[39., 40.],
#            [11., 33.],
#            [10., 4.],
#            [40., 19.]]

    xmin,ymin = list(np.min(POLY,axis=0))
    xmax,ymax = list(np.max(POLY,axis=0))

    xmid = (xmin+xmax) /2.
    ymid = (ymin+ymax) /2.

    #center the polygon to have its middle be [0,0]
    POLY = map(lambda p: [p[0]-xmid, p[1]-ymid], POLY)

    #shifting the center does not affect lengths
    xsize = gm.euclid_distance([xmax,ymid],[xmin,ymid])
    ysize = gm.euclid_distance([xmid,ymax],[xmid,ymin])

    WINDOW_METERS = max(xsize,ysize)

else:
    from geopy.distance import vincenty
#    in GPS minutes, will be converted to meters
#    POLY = np.array([[59.360307, 18.052161],
#                     [59.358581, 18.052697],
#                     [59.359098, 18.054961],
#                     [59.360558, 18.054811]])
#    POLY = np.array([[59.349175, 18.071001],
#                     [59.348092, 18.071087],
#                     [59.348065, 18.073040],
#                     [59.349124, 18.072647]])

    POLY = np.array([[59.359856, 18.054179],
                     [59.359852, 18.052800],
                     [59.359531, 18.052868],
                     [59.359581, 18.054383]])

    latmin,lonmin = list(np.min(POLY,axis=0))
    latmax,lonmax = list(np.max(POLY,axis=0))

    latmid = (latmin+latmax) /2.
    lonmid = (lonmin+lonmax) /2.

    latsize = vincenty([latmax,lonmid],[latmin,lonmid]).meters
    lonsize = vincenty([latmid,lonmax],[latmid,lonmin]).meters

    WINDOW_METERS = max(latsize,lonsize)

#ASSUME the earth is flat on for this polygon, dont care about the spherical shape
# map lat,lon to x,y 1:1.

    lat_scaled = u.scale_range(POLY[:,0], -WINDOW_METERS/2., WINDOW_METERS/2.)
    lon_scaled = u.scale_range(POLY[:,1], -WINDOW_METERS/2., WINDOW_METERS/2.)

    lat_to_y = lambda lat: u.scale_range([lat],
                                         -WINDOW_METERS/2.,
                                         WINDOW_METERS/2.,
                                         org_min = latmin,
                                         org_max = latmax)

    lon_to_x = lambda lon: u.scale_range([lon],
                                         -WINDOW_METERS/2.,
                                         WINDOW_METERS/2.,
                                         org_min = lonmin,
                                         org_max = lonmax)

    LATLON_TO_XY = lambda latlon: [lon_to_x(latlon[1]),
                                   lat_to_y(latlon[0])]


    y_to_lat = lambda y: u.scale_range([y],
                                       latmin,
                                       latmax,
                                       org_min = -WINDOW_METERS/2.,
                                       org_max = WINDOW_METERS/2.
                                       )

    x_to_lon = lambda x: u.scale_range([x],
                                       lonmin,
                                       lonmax,
                                       org_min = -WINDOW_METERS/2.,
                                       org_max = WINDOW_METERS/2.
                                       )

    XY_TO_LATLON = lambda xy: [y_to_lat(xy[1]),
                               x_to_lon(xy[0])]
###############################################################################
# FILES
###############################################################################
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
INIT_POS = [[-3.,0.],[3.,0.]]
#starting headings of agents in degrees. ccw positive from x axis
INIT_HEADING = [180.,0.]
#max forward speed and angular speeds
AGENT_MAX_V = 5 #m/s 1.5 normal
AGENT_MAX_W = 40. #degrees 15 normal
#simulation updates per second
UPDATE_FPS = 60.
#simulation area size. Middle point is 0,0, this value is the length of one side
#of the square area
WINDOW_METERS = WINDOW_METERS #1k final
#should the agent ignore broadcasts that are far away?
SIMULATE_NETWORK_BREAKAGE = False
#range to start ignoring messages in meters
NETWORK_RANGE = 10


###############################################################################
# TAHIROVIC
###############################################################################
# n x n grid of sampled points.
T_DENSITY = 200
# use variance to weight the centroid for tahirovic?
T_WEIGHTED = True
#radius to assume an area is explored
TVIC_RADIUS = 10


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
#max. amount of std.dev. to consider a point 'explored', above this, the point
#will be considered 'unexplored'
#MIN_STD = 0.13
#MIN_STD = 0.09
#discount points that will take longer to reach? this is the divider in utility func.
CARE_ABOUT_TTR = True
#should the agent avoid land?
AVOID_LAND = False

#roughly number of points to use when fitting GP. This is increased near the end.
DEFAULT_ENOUGH = 165

#velocity obstacle agent disc radius assumptions. Keep in mind that there is a delay
#in communications, so the 'other' radius should probably be larger than self radius
#in meters.
VO_SELF_R = 2.
VO_OTHER_R = 5.

#limit of depth to explore. Shallower (d<limit) points are considered unreachable
#or land. in meters.
DEPTH_LIMIT = 2.

#number of measurements expected to start using gp
MEASUREMENT_COUNT_THRESHOLD = 5
#min number of points to expect to consider choosing one
MIN_UNEXPLORED = 1
#default values of smallest search donut. in meters

DEFAULT_CIRCLE_COUNT = 120
if USE_PURE_TVIC:
    #use a tighter donut for this, cuz we can
    DEFAULT_START_RANGE = 5
    DEFAULT_END_RANGE = 12
    SEARCH_INCREMENT = 5
else:
    DEFAULT_START_RANGE = 20 #TODO decrease this
    DEFAULT_END_RANGE = 30
    #incerement in range when no points is found
    SEARCH_INCREMENT = 10
#time to wait between broadcasting missing values in seconds
MISSING_INTERVAL = 5
#dont flood udp buffers with the same mment, this is for 'real-time' bcasting
#of x,y,vx,vy,d. in seconds
MMENT_INTERVAL = 0.5
#allowed proximity of agent targets to each other when searching for
#the next point to go. in meters.
TARGET_PROXIMITY_LIMIT = 15


#distance limit to record measurements for agents. Not moving or moving too slowly
#causes instabilities for GP.
DISTANCE_THRESHOLD = 0.3


#distance threshold to consider a target reached
if SIMULATION:
    #in meters
    TARGET_DISTANCE_THRESHOLD = 1.
else:
    #in meters
    TARGET_DISTANCE_THRESHOLD = 1.

#angle threshold to consider a target 'in front'. In degrees.
#only used in simulations
TARGET_ANGLE_THRESHOLD = 0.5


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
    PAINT_TAHIROVIC = False
    PAINT_LAND = False
else:
    PAINT_EXPLORED = False
    PAINT_UNEXPLORED = False
    PAINT_TAHIROVIC = False
    PAINT_LAND = False
#draw the tahirovic centroid on the canvas?
T_DRAW_CENTROID = True
#display size in inches of screen
WINDOW_INCHES = 5.
#window dpi
WINDOW_DPI = WINDOW_SIZE/WINDOW_INCHES

###############################################################################
# DERIVED VALUES
###############################################################################
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