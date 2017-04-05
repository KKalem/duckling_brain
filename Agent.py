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


def time_to_traverse_arc(d, phi, max_v, max_w):
    """
    calculate the time, angular speed, forward speed, to traverse the arc that
    conencts to a point d meters away that is phi angles off-course
    d in meters, phi in radians.
    """
    #length of the arc between self and point
    arc_len = d*phi/np.sin(phi)

    #given this arc, what should the angular speed of the agent be to follow it
    #see paper for its derivation
    w = (2*phi*max_v*max_w) / ((arc_len*max_w)+(2*phi*max_v))
    #the linear speed agent can maintain while turning
    v = max_v*(1- w/max_w)
    #time to reach point over the arc given linear speed
    t = arc_len/v

    return (w,v,t)


class Measurement:
    """
    A simple container class to hold times and sequences of a measurement
    """
    def __init__(self, sensor_type):
        """
        type -> type of sensor, mostly used for accessing etc
        xxx_int. -> interval of time/pos/etc to ignore new values
        """
        #type of sensor
        self.type = sensor_type
        #last taken measurement, to avoid dict traversing
        if self.type != 'gps':
            self.last_value = -1
        else:
            self.last_value = [-1,-1,-1,-1]
        #time of last measurement
        self.last_time = -1
        #last position of recording
        self.last_pos = (-1,-1)
        #dictionary of time:measurement to hold all measurements in time
        self.values_by_time = {self.last_time:self.last_value}
        #dictionary of time:measurement to hold all measurements space
        self.values_by_pos = {self.last_pos:self.last_value}


    def add_value(self, value, t, pos):
        """
        add a new value to the dict.
        t is time of recording
        pos is position of recording
        """
        #check if the new value is actually something sensible
        #value itself can be none
        if value is None:
            return
        #value can be a list containing a none
        if type(value) == type([]):
            nones = filter(lambda e: e is None, value)
            if len(nones):
                return

        #all good!
        d_pos = config.DISTANCE_THRESHOLD

        if self.last_pos is not None:
            #gps has a special condition where pos makes no sense
            if self.type == 'gps':
                d_pos = gm.euclid_distance(self.last_value, value)
            else:
                d_pos = gm.euclid_distance(self.last_pos, pos)

        if d_pos >= config.DISTANCE_THRESHOLD:
            self.last_value = value
            self.last_time = t
            self.values_by_time[t] = value
            self.values_by_pos[tuple(pos)] = value
            self.last_pos = pos

    def get_by_pos(self, pos):
        """
        convenience func to get values from a list instead of a tuple
        """
        return self.values_by_pos.get(tuple(pos))

    def get_time_series(self):
        """
        return time,value pairs, sorted by time
        """
        times = self.values_by_time.keys()
        for t in sorted(list(times)):
            yield t, self.values_by_time.get(t)



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

        #sensor names for this agent, network doesnt need a buffer-like system like this
        sensor_types = ['gps','energy','sonar']
        #containers for the sensor readings
        self.sensors = {}
        for sensor in sensor_types:
            self.sensors[sensor] = Measurement(sensor)
        self.sensors['gps'].last_value = [-1,-1,-1,-1] #because vectors

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
            # draw the gps values on the display
            x = self.sensors['gps'].last_value[0] * config.CONTROL_PPM
            y = self.sensors['gps'].last_value[1] * config.CONTROL_PPM
            vx = self.sensors['gps'].last_value[2] * 20.
            vy = self.sensors['gps'].last_value[3] * 20.
            l = g.Line(g.Point(x,y),g.Point(x+vx,y+vy))
            l.setArrow('last')
            l.draw(self.win)

            #draw the sonar values
            d = self.sensors['sonar'].last_value + 10
            cir = g.Circle(g.Point(x,y), 5)
            color = g.color_rgb(255-(d*4),150, d*4)
            cir.setFill(color)
            cir.setOutline(color)
            cir.draw(self.win)
        except:
            pass

    def draw_other(self, agent_id):
        """
        draws the information gained from another agent on self display
        """
#        agent = self.others.get(agent_id)
        #TODO make self.others from network inputs
        pass


    def _make_debug_text(self):
        txt = ''
        try:
            t = []
            t.append('tim;'+u.float_format2(self.time))
            t.append('snr;'+u.float_format2(self.sensors['sonar'].last_value))
            t.append('pos;'+u.float_format2(self.sensors['gps'].last_value[0])+','+
                            u.float_format2(self.sensors['gps'].last_value[1]))
            t.append('vel;'+u.float_format2(self.sensors['gps'].last_value[2])+','+
                            u.float_format2(self.sensors['gps'].last_value[3]))

            t.append('fuel;'+u.float_format2(self.sensors['energy'].last_value))
            t.append('heading;'+u.float_format2(self.heading[0])+','+
                                u.float_format2(self.heading[1]))

            txt = ''
            for field in t:
                txt += field+'\n'
        except:
            import traceback
            traceback.print_exc()
        return txt

    def make_measurement_list(self):
        """
        coaleces the relevant gps and sonar measurements into a list for GP
        including the measurements from other agents
        """
        #TODO instead of putting in all the measurements, choose the closest ones
        #from the quadtree instead. This will cut greatly on GP processing time.
        measurements = []
        #self measurements
        for pos in self.sensors['gps'].values_by_time.values():
            depth = self.sensors['sonar'].get_by_pos(pos[:2])#dont need velocities
            if not (pos[0]==pos[1]==-1 or pos[0]==pos[1]==0):
                measurements.append([pos[0],pos[1],depth])

        #measurements from other agents
        for agent in self.other_agents:
            #TODO return here when other agents stuff is implemented
            pass
        return measurements

    def time_to_point(self, point):
        """
        How long will it take to reach a given point in the map, given the current
        position, velocity etc.
        """
        #TODO implement
        pass

    def sample_circle(self, radius):
        """
        returns a list of points that form a circle around the agent
        """
        #TODO implement
        pass

    def evaluate_points(self, points, obj_fun):
        """
        returns a point with maximum value from the given points.
        values are determined by some objective function
        """
        #TODO impl
        pass

    def gp_objective(self, point):
        """
        returns a value for a given point depending on GP variance estimate
        """
        #TODO impl
        pass

    def time_objective(self, point):
        """
        returns a value for a given point depending on time to reach it

        see ~/test.py to see how arcing compares to turn+forward movement.
        see your paper for derivations.

        TLDR: arcing is never faster given speed decrease when turning.
        This actually makes things easier for me in terms of waypointing and such
        """
        #gps sensor
        gps = self.sensors['gps']
        #current global pos. and heading of self
        pos = np.array(gps.last_value[:2])
        heading = self.heading

        #relative position of the point to self
        rel_point = np.array(point) - pos
        #line distance to point from self
        d = gm.euclid_distance(point, pos)
        #vector angle between self heading and point
        phi = gm.vec_angle(rel_point, heading)

        #parameters of the agent body
        max_v = config.AGENT_MAX_V
        max_w = u.to_rad(config.AGENT_MAX_W)
        #if we were to go in an arc to the target...
        arc_w,arc_v,arc_t = time_to_traverse_arc(d,phi,max_v,max_w)

        #if we were to turn first, then go forward....
        #time to turn+time to go forward
        str_t = phi/max_w + d/max_v

        times = [arc_t, str_t]

        return np.argmin(times)==0, np.min(times)


    def get_target(self):
        #TODO impl
#        radius = 3
#        points = self.sample_circle(radius)
#        values = self.evaluate_points(points, self.gp_objective, self.time_objective ....)
#        target = np.argmax(values)
#        return target
        pass



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
                    else:
                        #received some measurement, add to buffers if needed.
                        #see Measurement.add_value for details
                        self.sensors[sensor_type].add_value(sensor_value,
                                                            self.time,
                                                            self.sensors['gps'].last_value[:2])
                        #also record the heading if the value makes sense
                        if sensor_type == 'gps':
                            vel = sensor_value[2:]
                            speed = gm.euclid_distance([0,0],vel)
                            if speed > 0.5:
                                self.heading = vel


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
            for sensor in self.sensors.values():
                print(sensor.type,len(sensor.values_by_time))
        if key == 't':
            #print time series of sensors
            for sensor in self.sensors.values():
                print(sensor.type)
                for t,val in sensor.get_time_series():
                    print(t,val)
            #save the traces too
            self.save_trace()
        if key == 'g':
            #draw the GP regressed surface
            if len(self.sensors['gps'].values_by_time) > 5:
                means, stds = self.gp.fit_regress(None, self.make_measurement_list())
            else:
                print('move around a little first, not enough measurements!',
                      len(self.sensors['gps'].values_by_time))

        mouse = self.win.checkMouse()
        if mouse is not None:
                x = mouse.getX()/config.CONTROL_PPM
                y = mouse.getY()/config.CONTROL_PPM
                print('mouse_meters;',u.float_format2(x),u.float_format2(y))
                do_arc, t = self.time_objective([x,y])
                print('do_arc',do_arc,'time',t)

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
        gps = self.sensors['gps']
        sonar = self.sensors['sonar']
        gps_trace = gps.get_time_series()
        sonar_trace = sonar.get_time_series()
        with open(config.TRACE_DIR+self.id+'_gps_trace','w') as f:
            for t,pos in gps_trace:
                f.write(str(t)+' '+str(pos[0])+' '+str(pos[1])+'\n')
        with open(config.TRACE_DIR+self.id+'_sonar_trace','w') as f:
            for t,d in sonar_trace:
                f.write(str(t)+' '+str(d)+'\n')

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
