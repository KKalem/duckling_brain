# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 19:10:08 2017

@author: ozer
"""
from __future__ import print_function

import udp
import config
import util as u

import time

import numpy as np


class State:
    """
    A simple state object to store values
    """
    def __init__(self, id):
        # all meter seconds and degrees
        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.
        self.ax = 0.
        self.ay = 0.


        self.speed = 0. # scalar, forward speed of the body m/s
        self.accel = 0. # scalar, forward acceleration of the body m/s^2
        self.turn = 0. #scalar, turning speed of the body. deg/s
        self.heading = 0. #from y=0 line, in degrees.

        self.id = id

        #percentage and percent per second
        self.fuel = 100.

    def __str__(self):
        return str(['pos',self.x,self.y,
                    'spd',self.speed,
                    'acc',self.accel,
                    'turn',self.turn,
                    'vel',self.vx,self.vy,
                    'fuel',self.fuel])


class DynamicBody:
    """
    A dynamic body with accelarion and stuff
    kwargs:
        -base -> a pseudo-agent base object
        -'depthmap' -> a map of the seabed, 2d numpy array
        -other_bodies -> a list of Body's
        -'x','y','vx','vy','ax','ay','pow','speed','accel','turn'
            -> all optional, assumed to be 0 if not given
        -'max_turn' -> turning rate, assumed 15 deg/s if not given
        -'max_spd' -> maximum forward speed of the vehicle
    """
    def __init__(self, id, **kwargs):
        #string identifier for this body. should have a matching id for its agent
        self.id = id
        #string address for this body. Messages sent to this body should be addressed to this
        self.addr = self.id+'-body'
        #the base station
        self.base = kwargs.get('base')
        #list of bodies of agents, to be used for determining range stuffs
        self.other_bodies = kwargs.get('other_bodies', [])
        #init time
        self.last_time = time.time()
        #max turnation speed, degree per sec
        self.max_turn = kwargs.get('max_turn', 15.)
        #max forward speed.
        self.max_speed = kwargs.get('max_speed', 1.5)

        #the depth map of the world
        self.depthmap = kwargs.get('depthmap')


        #a target linear speed towards the heading
        self.target_speed = 0.

        #initial state
        self.state = State(self.id)

        #udp packet sender this agent will use
        self.producer = udp.producer(sender_ip = config.HOST_IP,
                                     sender_port = config.SENDER_PORT,
                                     ttl = config.TTL)

        #udp packet receiver this agent will use
        self.consumer = udp.consumer(client_ip = config.HOST_IP,
                                     mcast_addr = config.MCAST_ADDR,
                                     mcast_port = config.MCAST_PORT,
                                     ttl = config.TTL,
                                     blocking = 0)

        #bunch of state points.
        #pow is the current power percentage
        #speed and accel are scalar with respect to heading
        state_elems = ['x','y','vx','vy','ax','ay','pow','speed','accel','turn']
        for elem in state_elems:
            setattr(self.state, elem, kwargs.get(elem, 0.))


        #names of sensor handling functions
        sensors = [config.GET_GPS,config.GET_ENERGY,config.GET_SONAR,config.GET_NETWORK]
        #last times each sensor responded. Will be used to limit rates
        self.last_times = {}
        for sensor in sensors:
            self.last_times[sensor] = time.time()

    def _to_agent(self, msg):
        self.producer.send(u.msg(self.id+'-agent', msg))

    def _to_agent_sensor(self, sensor_type, msg):
        self.producer.send(u.msg(self.id+'-'+sensor_type, msg))


##############################################################################
# Handler functions for commands from agent proc.
##############################################################################
    def get_gps(self):
        #check if sufficient time has passed since last poll
        dt = time.time() - self.last_times[config.GET_GPS]
        if  dt > config.GPS_FREQ:
            output = [self.state.x, self.state.y, self.state.vx, self.state.vy]
            #add noise
            output[0] += u.rand_range(config.GPS_POS_NOISE)
            output[1] += u.rand_range(config.GPS_POS_NOISE)
            self._to_agent_sensor('gps', output)
            self.last_times[config.GET_GPS] = time.time()

    def get_nrg(self):
        dt = time.time() - self.last_times[config.GET_ENERGY]
        if dt > config.NRG_FREQ:
            output = self.state.fuel
            self._to_agent_sensor('energy', output)
            self.last_times[config.GET_ENERGY] = time.time()

    def get_snr(self):
        dt = time.time() - self.last_times[config.GET_SONAR]
        if dt > config.SNR_FREQ:
            output = self.depthmap.get_depth_px(self.state.x * config.PPM, self.state.y * config.PPM)
            #add noise
            output += u.rand_range(config.SONAR_NOISE)
            self._to_agent_sensor('sonar', output)
            self.last_times[config.GET_SONAR] = time.time()

    def get_net(self):
        dt = time.time() - self.last_times[config.GET_NETWORK]
        if dt > config.NET_FREQ:
            #TODO proper network connections etc
            output = '###net-'+u.float_format2(time.time())
            self._to_agent_sensor('network', output)
            self.last_times[config.GET_NETWORK] = time.time()

    def inc_speed(self):
        self.target_speed += 0.1

    def dec_speed(self):
        self.target_speed -= 0.1

    def turn_left(self):
        self.state.turn += 0.5

    def turn_right(self):
        self.state.turn -=0.5

    def set_max_speed(self):
        self.target_speed = 10.

    def set_stop(self):
        self.target_speed = 0.

    def reset_turn(self):
        self.state.turn = 0.

    def set_target(self, point):
        #TODO implement going to target
        pass


    def receive_and_run(self, sim_time):
        """
        This physical body reads sensor requests and responds with the values
        """
        messages = u.msgs_to_self(self.addr, self.consumer)
        messages = set(messages) #get rid of duplicates
        if len(messages) > 0:
#            print('### '+self.addr+' received:', messages, 'on time:',sim_time)
            for message in messages:
                #if the body has a function named the same as message, call it
                if hasattr(self, message):
                    handler_function = getattr(self, message)
                    handler_function()
                else:
                    print('### '+self.addr+' unknown function call:',message)


    def update(self, sim_time):
        """
        updates the physical properties of this body
        dt is delta time since last update in seconds
        all values in meters and seconds
        energy is percent left
        pow is percent per second
        """
        if self.last_time < 1.:
            self.last_time = time.time()



        #time passed since last update
        dt = time.time() - self.last_time
        #convenience
        s = self.state

        #limit turning rate to [0,max rate]
        s.turn = u.clamp(-self.max_turn,self.max_turn,s.turn)
        #keep heading in [0,360)
        s.heading = s.heading %360
        #the speed of the agent on its heading
        s.speed = s.vx * u.cos(s.heading) + s.vy * u.sin(s.heading)

        #need to apply accel?
        if s.speed < self.target_speed:
            #forward accel that will be applied
            s.accel = 0.5 - 0.222*(s.speed**2)
        else:
            # no reversing and braking=not accelarating
            s.accel = 0.


        #######################################################################
        #physics!
        #######################################################################
        #change in heading
        s.heading += s.turn *dt

        #apply acceleration
        s.speed += s.accel * dt

        #maximum allowed speed w/respect to turn rate
        allowed_speed = self.max_speed * (self.max_turn - np.abs(s.turn))/self.max_turn

        #if turning, regardless of accel, speed is lowered
        s.speed = min(allowed_speed, s.speed)

        #if no accel is applied, drift to a halt
        if s.accel == 0.:
            if s.vx < 0.02:
                s.vx = 0
            else:
                s.vx -= 0.222*(s.vx**2)
            if s.vy < 0.02:
                s.vy = 0
            else:
                s.vy -= 0.222*(s.vy**2)
        else:
            #change in velocity
            s.vx = s.speed * u.cos(s.heading)
            s.vy = s.speed * u.sin(s.heading)

        #apply world-coord speed to displacement
        dx = s.vx * dt
        dy = s.vy * dt
        s.x += dx
        s.y += dy


        #fuel usage
        s.fuel -= 20.*(s.speed**2)/60 * dt

        #record time for next time
        self.last_time = time.time()
        #return the update for graphics
        return dx, dy
