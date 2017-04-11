#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 21:35:28 2017

@author: ozer
"""
from __future__ import print_function

import udp
import config
import util as u
import graphics as g
import sensors
import geometry as gm
import gp

import sys
import traceback
import time
import numpy as np
from subprocess import Popen


class Agent:
    """
    An agent that will control a body in simulation through some comm. channel
    This object does not store data that it did not receive over the comm.
    channel.
    """
    def __init__(self, id):
        """
        initialize the agent, id defines the body it will control
        creates a small info and control window
        """
        #id and address of the agent. Should have a corresponding body and sensors
        self.id = id
        self.addr = self.id+'-agent'

        #keep track of agent time since conception
        self.time = 0.
        self.start_time = time.time()

        #remote or auto
        #remote for getting keyboard input to control speed/turn
        self.mode = 'remote'

        #TODO devise some method of storing information from other agents
        # also look at make_measurement_list when done
        #this ties into the network stuffs
        self.other_agents = []

        #the GP object for this agent
        self.gp = gp.GP()

        #last known heading from the gps. This is useful if/when the agent is
        #not moving forward at all.
        self.heading = (-1,-1)

        ######################################################################
        # sensor stuff
        ######################################################################
        #spawn commands for the sensor processes
        spawn_commands = [
        sensors.make_spawn_command(self.id, 'gps', config.GET_GPS, config.GPS_POLL),
        sensors.make_spawn_command(self.id, 'energy', config.GET_ENERGY, config.NRG_POLL),
        sensors.make_spawn_command(self.id, 'sonar', config.GET_SONAR, config.SNR_POLL),
        sensors.make_spawn_command(self.id, 'network', config.GET_NETWORK, config.NET_POLL)
        ]

        #start the sensors this agent will take readings from
        self.sensor_processes = []
        for command in spawn_commands:
            proc = Popen(command)
            self.sensor_processes.append(proc)

        #a list of (x,y,d) for sensor/gps measurements.
        #x,y or d can be None or repeated if sensor data is missing for some reason
        self.measurements = []
        #a list of integer IDs for each measurement. This is used when merging
        #with other agents
        self.m_ids = []

        #last known velocity of the body (vx,vy)
        self.V = (0.,0.)



        ######################################################################
        # display
        ######################################################################
        #a window to draw stuff about this agent
        self.win = g.GraphWin('Agent:'+str(self.id), config.CONTROL_WIN_SIZE, config.CONTROL_WIN_SIZE)
        self.win.setCoords(-config.CONTROL_WIN_SIZE/2., -config.CONTROL_WIN_SIZE/2.,
                      config.CONTROL_WIN_SIZE/2., config.CONTROL_WIN_SIZE/2.)
        self.win.setBackground(g.color_rgb(220,250,255))

        #some debug text to display on the control window
        self.debugtext = g.Text(g.Point(-config.CONTROL_WIN_SIZE/2.+70,0),'No-text')
        self.debugtext.draw(self.win)


        ######################################################################
        # comms
        ######################################################################
        #agent communication channels
        self.producer = udp.producer()
        self.consumer = udp.consumer()


    def draw_self(self):
        """
        draws a representation of the agents own state
        """
        try:
            i = 0
            max_i = len(self.measurements)
            nones = [0]
            while i<max_i and len(nones)>0:
                i+=1
                x,y,d = self.measurements[-i]
                nones = filter(lambda v: v is None, [x,y,d])

            vx,vy = self.V
            # draw the gps values on the display
            x *= config.CONTROL_PPM
            y *= config.CONTROL_PPM
            vx *= 20.
            vy *= 20.
            l = g.Line(g.Point(x,y),g.Point(x+vx,y+vy))
            l.setArrow('last')
            l.draw(self.win)

            #draw the sonar values
            d += 10
            cir = g.Circle(g.Point(x,y), 5)
            color = g.color_rgb(255-(d*4),150, d*4)
            cir.setFill(color)
            cir.setOutline(color)
            cir.draw(self.win)
        except:
            pass


###############################################################################
# actual control that is run continuesly
###############################################################################
    def control(self):
        #time since starting control
        self.time = time.time() - self.start_time

        #draw the agent on display
        self.draw_self()

        ######################################################################
        # sensor readings
        ######################################################################
        #from the incoming queue, only read the ones addressed to this agent
        messages = u.msgs_to_self(self.addr, self.consumer)
        #if there are messages to this agent, handle'em
        if len(messages) > 0:
            for message in messages:
                sensor_type = message.get('type')
                sensor_value = message.get('value')
                if sensor_value is not None and sensor_type is not None:
                    if sensor_type == 'network':
                        #TODO handle network readings
                        pass
                    if sensor_type == 'gps':
                        #velocity and heading from gps
                        self.V = sensor_value[2:]
                        speed = gm.euclid_distance([0,0],self.V)
                        if speed > 0.1:
                            self.heading = self.V

                        #position from gps
                        x,y = sensor_value[:2]
                        #do we have a previous measurement?
                        if len(self.measurements) > 0:
                            old_x, old_y, _ = self.measurements[-1]
                            #does this measurement have a position added?
                            if old_x is None or old_y is None:
                                #it needs a position, add it
                                self.measurements[-1][0] = x
                                self.measurements[-1][1] = y
                            else:
                                #it doesnt need a position
                                #is the new point far enough away?
                                distance = gm.euclid_distance([old_x,old_y],[x,y])
                                if distance >= config.DISTANCE_THRESHOLD:
                                    self.measurements.append([x,y,None])
                                    new_id = self.m_ids[-1] +1
                                    self.m_ids.append(new_id)
                        else:
                            #no previous measurements, this is the first
                            self.measurements.append([x,y,None])
                            self.m_ids.append(0)

                    if sensor_type == 'sonar':
                        d = sensor_value
                        #any previous measurements?
                        if len(self.measurements) > 0:
                            #does the last added gps need a depth?
                            _, _, old_d = self.measurements[-1]
                            if old_d is None:
                                #its missing depth, add it
                                self.measurements[-1][2] = d
#                            else:
#                                #it needs nothing more, add a new measurement
#                                self.measurements.append([None,None,d])
#                                new_id = self.m_ids[-1] +1
#                                self.m_ids.append(new_id)


        #TODO do something with the data
        #set the debug text to sensor readings
        self.debugtext.setText(self._make_debug_text())


        ######################################################################
        # controls
        ######################################################################
        key = self.win.checkKey()
        #end control
        if key=='Escape':
            return True
        #switch between remote and autonomous controls
        if key == 'a':
            self.mode = 'auto'
        if key == 'r':
            self.mode = 'remote'
        if key == 'p':
            #print sensor dump
            print('measurements;',len(self.measurements))
        if key == 't':
            #save the traces
            self.save_trace()
        if key == 'g':
            #draw the GP regressed surface
            if len(self.measurements) > 5:
                means, stds = self.gp.fit_regress(None, self.measurements)
            else:
                print('move around a little first, not enough measurements!',
                      len(self.measurements))

        mouse = self.win.checkMouse()
        if mouse is not None:
                x = mouse.getX()/config.CONTROL_PPM
                y = mouse.getY()/config.CONTROL_PPM
                print('mouse_meters;',u.float_format2(x),u.float_format2(y))
                self.send_to_body('set_target'+','+str(x)+','+str(y))

        #send new commands to simulation/hardware
        #remote contrl
        if self.mode == 'remote':
            #the sent strings should have same-named methods with no arguments
            #in body. def inc_speed(self): .....
            if key=='Up':
                self.send_to_body('inc_speed')
            if key=='Down':
                self.send_to_body('dec_speed')
            if key=='s':
                self.send_to_body('set_stop')
            if key=='w':
                self.send_to_body('set_max_speed')
            if key=='Left':
                self.send_to_body('turn_left')
            if key=='Right':
                self.send_to_body('turn_right')
            if key=='x':
                self.send_to_body('reset_turn')

        #autonomous control
        else:
            print('### IMPLEMENT AUTO CONTRL',self.time)

        return False


    def send_to_body(self,msg):
        """
        convenience method to send requests to the body
        should be used to control the body of the agent
        """
        self.producer.send(u.msg(self.id+'-body',msg))


    def save_trace(self):
        """
        save the gps and sonar measurements to a file for reasons
        """
        with open(config.TRACE_DIR+self.id+'_trace','w') as f:
            for x,y,d in self.measurements:
                f.write(str(x)+','+str(y)+','+str(d)+'\n')

    def _make_debug_text(self):
        t = '-'
        try:
            i = 0
            max_i = len(self.measurements)
            nones = [0]
            while i<max_i and len(nones)>0:
                i+=1
                x,y,d = self.measurements[-i]
                nones = filter(lambda v: v is None, [x,y,d])

            vx,vy = self.V
            t = ''
            t += 'x,y,d; '+u.float_format2(x)+','+u.float_format2(y)+','+u.float_format2(d)+'\n'
            t += 'vx,vy; '+u.float_format2(vx)+','+u.float_format2(vy)+'\n'
            t += 'ms; '+str(len(self.measurements))+'\n'
        except:
            return t
        return t


if __name__=='__main__':
    try:
        id = sys.argv[1]
    except:
        id = '0'
    agent = Agent(id)
    try:
        done = False
        print('[I] Running')
        while not done:
            done = agent.control()
    except:
        traceback.print_exc(file=sys.stderr)
        print('[I] Stopped')
        agent.win.close()
        for proc in agent.sensor_processes:
            proc.kill()
    else:
        print('[I] Stopped')
        agent.win.close()
        for proc in agent.sensor_processes:
            proc.kill()
