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


###############################################################################
# Simulation
###############################################################################
#simulation updates per second
UPDATE_FPS = 60.
#display size in pixels
WINDOW_SIZE = 600
#display size in sim. meters
WINDOW_METERS = 200. #1k final
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
CONTROL_WIN_SIZE = 400
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

SET_TARGET = 'set_tgt'

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