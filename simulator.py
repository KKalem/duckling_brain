#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 13:11:55 2017

@author: ozer

Simulator for the roboat agents
"""
from __future__ import print_function

import graphics as g
import config
import udp
import util as u
from Body import DynamicBody
from depthmap import Depthmap

import sys
import traceback
import time
from subprocess import Popen

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

def draw_body(win, body, dx, dy, head):
    """
    Draws a simple graphic representing the body. dx,dy should be returned
    from its update method and should be difference in position since last
    update done.

    win is graphics.py graphwin
    body is the dynamicbody
    dx,dy is change in body's x,y
    head is the shape made and drawn on the previous update
    """
    #convert movement in meters to pixels
    pix_dx = dx*config.PPM
    pix_dy = dy*config.PPM
    #agent state
    s = body.state
    #draw the agent center
    agent_shapes[body.id].move(pix_dx,pix_dy)
    #draw the agent heading
    #heading as a point
    hx = u.cos(s.heading)*config.AGENT_SHAPE_RADIUS
    hy = u.sin(s.heading)*config.AGENT_SHAPE_RADIUS

    head.undraw()
    x = (s.x+hx)*config.PPM
    y = (s.y+hy)*config.PPM
    head = g.Circle(g.Point(x,y), config.AGENT_HEAD_RADIUS*config.PPM)
    head.setFill('red')
    head.draw(win)
    return head

def load_colormap(filename):
    """
    loads a simple matrix file as a colormap into a numpy array
    """
    v = []
    with open(filename) as f:
        for line in f:
            c = [int(part) for part in line.split()]
            v.append(c)
    m = np.array(v)/255.
    return m


###############################################################################
# MAIN
###############################################################################
if __name__=='__main__':
    print('[I] Simulator started')
    #main loop of the simulator, out of a function for ease of debugging
    try:
        #the physical bodies of the agents. These do not have any decision making
        #they are purely physics objects
        agent_bodies = {}
        #shapes of agents, purely graphics, for the center of their bodies
        agent_shapes = {}
        #shapes for the heads of agents.
        agent_heads = {}
        #process handles of agent controls
        agent_processes = {}

        #the base station that will be monitoring the whole process, not a real
        #agent, so it is separate.
        base = None

        #setup the canvas
        win = g.GraphWin('Simulation',config.WINDOW_SIZE,config.WINDOW_SIZE, autoflush=False)

        #make 0,0 center
        win.setCoords(-config.WINDOW_SIZE/2.,-config.WINDOW_SIZE/2.,
                      config.WINDOW_SIZE/2.,config.WINDOW_SIZE/2.)
        very_light_blue = g.color_rgb(220,250,255)
        win.setBackground(very_light_blue)

        print('[I] Generating depthmap')
        #generate a depthmap using perlin noise.
        depthmap = Depthmap(config.WINDOW_SIZE, config.WINDOW_METERS,
                            config.DEPTHMAP_OCTAVES, config.WINDOW_SIZE/2.,
                            config.DEPTHMAP_BASE, config.DEPTHMAP_SMOOTHING,
                            config.DEPTHMAP_MINDEPTH, config.DEPTHMAP_MAXDEPTH)

        print('[I] Saving depthmap')
        depthmap.save_depthmap()

        print('[I] Drawing depthmap')
        #create and save a nicely coloured depthmap
        colormap = mpl.colors.ListedColormap(load_colormap(config.COLORMAP_FILE))
        fig = plt.figure(frameon=False)
        fig.set_size_inches(config.WINDOW_INCHES, config.WINDOW_INCHES)
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        fig.add_axes(ax)
        ax.matshow(depthmap.map, cmap=colormap, aspect='equal',
                   vmin=config.DEPTHMAP_MINDEPTH, vmax=config.DEPTHMAP_MAXDEPTH,
                   origin='lower')
        fig.savefig(config.DEPTHMAP_FILE, dpi=config.WINDOW_DPI)

        #draw the depthmap onto the main window
        depth_img = g.Image(g.Point(0,0), config.DEPTHMAP_FILE)
        depth_img.draw(win)

        print('### min/max of map:',np.min(depthmap.map),np.max(depthmap.map))


        #init bodies and shapes
        for i in range(config.NUMBER_OF_AGENTS):
            #agent body
            strid = str(i)
            body = DynamicBody(id = strid, depthmap = depthmap)
            agent_bodies[strid] = body

            #shape of the body
            shape = g.Circle(g.Point(body.state.x, body.state.y), config.AGENT_SHAPE_RADIUS*config.PPM)
            shape.setFill('red')
            shape.draw(win) #draw this, it will be moved
            agent_shapes[strid] = shape

            #shape of the head, dont draw, it is re-drawn everyt frame, not moved
            head = g.Circle(g.Point(body.state.x+1, body.state.y+1),
                            config.AGENT_HEAD_RADIUS*config.PPM)
            agent_heads[strid] = head


        #draw the time on screen
        sim_time = 0
        timetext = g.Text(g.Point(0,-config.WINDOW_SIZE/2.+20), sim_time)
        timetext.draw(win)

        #some debug text, set at the end of the event loop if needed.
        debugtext = g.Text(g.Point(-config.WINDOW_SIZE/2.+70,0), '-')
        debugtext.draw(win)


        #give a reference to other agents, so they can determine if they are in
        #range of eachother and the like
        for body in agent_bodies.values():
            body.other_bodies = agent_bodies
            body.base = base


        #communication with everything is done over the same channel,
        #agents/simulation needs to read this cahnnel and filter out the
        #packets not meant for themselves.
        #udp packet sender the simulator will use
        producer = udp.producer(sender_ip = config.HOST_IP,
                                     sender_port = config.SENDER_PORT,
                                     ttl = config.TTL)

        #udp packet receiver the simulator will use
        consumer = udp.consumer(client_ip = config.HOST_IP,
                                     mcast_addr = config.MCAST_ADDR,
                                     mcast_port = config.MCAST_PORT,
                                     ttl = config.TTL,
                                     blocking = 0)


###############################################################################
# START CONTROLS
###############################################################################
        if config.START_CONTROLS:
            print('[I] Starting control processes')
            for body in agent_bodies:
                proc = Popen(['python', 'Agent.py', str(body.id)])
                agent_processes.append(proc)


###############################################################################
# EVENT LOOP
###############################################################################
        print('[I] Main loop running')
        start_time = time.time()
        while True:

            #some buttons to check for small things
            key = win.checkKey()
            if key=="Escape":
                break
            if key=='r':
                print('[I] reset')
                #TODO reset the sim here

            #for testing purposes, print the depth under mouse click
            mouse = win.checkMouse()
            if mouse is not None:
                x = mouse.getX()/config.PPM
                y = mouse.getY()/config.PPM
                print('mouse_meters;',u.float_format2(x),u.float_format2(y),
                      'depth:',u.float_format2(depthmap.get_depth(x,y)))


            #update the frame, this caps the simulation and drawing updates
            g.update(config.UPDATE_FPS)
            sim_time = time.time() - start_time
            #update agents
            for body in agent_bodies.values():
                #update physics
                #record the change
#                dx, dy = body.update(sim_time)
                dx, dy = body.update_simple(sim_time)

                #receive the commands generated from agents
                #these are the NMEA sentences meant for the physical bodies to act on
                #receive the sentence from agent
                #this command will take effect the next update
                body.receive_and_run(sim_time)

                #update the graphics
                if win is not None:
                    #draw the time on screen
                    timetext.setText("{0:.1f}".format(sim_time))
                    agent_heads[body.id] = draw_body(win, body, dx, dy, agent_heads[body.id])


                    #draw some debug text
                    s = agent_bodies['0'].state
                    debugtext.setText('vel:'+"{0:.3f}".format(s.vx)+",{0:.3f}".format(s.vy)+
                                    '\nspd:'+"{0:.3f}".format(s.speed)+
                                    '\nacc:'+"{0:.3f}".format(s.accel)+
                                    '\nturn:'+"{0:.3f}".format(s.turn)+
                                    '\nhdng:'+"{0:.3f}".format(s.heading)+
                                    '\nfuel:'+"{0:.3f}".format(s.fuel)+
                                    '\ntspd:'+"{0:.3f}".format(agent_bodies['0'].target_speed))

    except:
        #either something threw an exception or a sigkill was sent
        traceback.print_exc(file=sys.stderr)
        print('[I] Simulator stopped')
        win.close()
        for proc in agent_processes:
            proc.kill()
    else:
        #done with the simulation normally
        print('[I] Simulator stopped')
        win.close()
        for proc in agent_processes:
            proc.kill()

