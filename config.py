# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 16:15:43 2017

@author: ozer

Bunch of global values for other stuff to read from.
"""

#how many mobile agents will be simulated
NUMBER_OF_AGENTS = 1

#agent shape radius, in meters
AGENT_SHAPE_RADIUS = 1.5

#agent head shape radius
AGENT_HEAD_RADIUS = 0.5

#simulation updates per second
UPDATE_FPS = 60

#display size in pixels
WINDOW_SIZE = 800

#display size for agent control windows
CONTROL_WIN_SIZE = 300

#should the simulation start the control processes or will they be started manually?
START_CONTROLS = False

#display size in meters, top left is x=0, max x would be this
WINDOW_METERS = 50

#display pixels per meter
PPM = WINDOW_SIZE / WINDOW_METERS*1.

#friction coefficient for water
FRICTION = 0.3

###############################################################################
# UDP STUFF
###############################################################################
#use all interfaces on the port
HOST_IP = "0.0.0.0"
#ports where everything will dump their data into
SENDER_PORT = 1501
#multicast params
MCAST_ADDR = "224.168.2.9"
MCAST_PORT = 1600
TTL = 31# valid value are 1-255, <32 is local network
