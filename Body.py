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
        -other_bodies -> a list of Body's
        -'x','y','vx','vy','ax','ay','pow','speed','accel','turn'
            -> all optional, assumed to be 0 if not given
        -'max_turn' -> turning rate, assumed 15 deg/s if not given
    """
    def __init__(self, id, **kwargs):
        #string identifier for this body. should have a matching id for its agent
        self.id = id
        #string address for this body. Messages sent to this body should be addressed to this
        self.addr = self.id+'-body'
        #the base station
        self.base = kwargs.get('base')
        #list of bodies of agents, to be used for determining range stuffs
        self.other_bodies = kwargs.get('other_bodies') if kwargs.get('other_bodies') else []
        #init time
        self.last_time = time.time()
        #max turnation speed, degree per sec
        self.max_turn = kwargs.get('max_turn') if kwargs.get('max_turn') else 15.
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
            setattr(self.state, elem, kwargs.get(elem) if kwargs.get(elem) else 0.0)

    def _to_agent(self, msg):
        self.producer.send(u.msg(self.id+'-agent', msg))

    def receive_and_run(self):
        """
        This physical body reads from the UDP ports that connect to its 'brain'
        and acts on the commands that are meant for it
        """
        messages = u.msgs_to_self(self.addr, self.consumer)
        if len(messages) > 0:
            print(self.addr+' received:', messages)
            for data in messages:
                if data=='Up':
                    self.target_speed += 0.1
                if data=='Down':
                    self.target_speed += -0.1
                if data=='Left':
                    self.state.turn += 5
                if data=='Right':
                    self.state.turn += -5
                if data=='s':
                    self._to_agent([self.state.x, self.state.y])



    def update(self):
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

        #keep heading in [0,360)
        s.heading = s.heading %360

        #the speed of the agent on its heading
        s.speed = s.vx * u.cos(s.heading) + s.vy * u.sin(s.heading)

        #clamp the target speed to 0
        if self.target_speed < 0.001:
            self.target_speed = 0.

        #this is given for the roboats.
        s.max_accel = 0.5 - 0.222*(s.speed**2)

        #target acceleration to reach the target speed
        target_accel = 0.
        #try to reach the desired speed
        if s.speed > self.target_speed + config.SPEED_TOLERANCE:
            target_accel = -s.max_accel
        if s.speed < self.target_speed + config.SPEED_TOLERANCE:
            target_accel = s.max_accel
        #ensure min/max accelarations
        s.accel = u.clamp(-s.max_accel, s.max_accel, target_accel)

        #update fuel
        fuel_consumption = (20*(s.speed**2))/3600. #fuel consumed per second
        s.fuel -= fuel_consumption * dt

        #clamp the turning rate
        s.turn = u.clamp(-self.max_turn,self.max_turn, s.turn)

        #pure physics updates
        #turn the body
        s.heading += s.turn * dt
        #forward accel to world coords
        s.ax = s.accel * u.cos(s.heading)
        s.ay = s.accel * u.sin(s.heading)

        #apply world-coord accel to speed
        s.vx += s.ax * dt
        s.vy += s.ay * dt

        #apply world-coord speed to displacement
        dx = s.vx * dt
        dy = s.vy * dt
        s.x += dx
        s.y += dy


        #record time for next time
        self.last_time = time.time()

        #return the update for graphics
        return dx, dy