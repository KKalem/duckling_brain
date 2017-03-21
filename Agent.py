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

import sys
import traceback
import time
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


        #a window to draw stuff about this agent
        self.win = g.GraphWin('Agent:'+str(self.id), config.CONTROL_WIN_SIZE, config.CONTROL_WIN_SIZE)
        self.win.setCoords(-config.CONTROL_WIN_SIZE/2., -config.CONTROL_WIN_SIZE/2.,
                      config.CONTROL_WIN_SIZE/2., config.CONTROL_WIN_SIZE/2.)
        self.win.setBackground('white')

        #some debug text to display on the control window
        self.debugtext = g.Text(g.Point(0,0),'No-text')
        self.debugtext.draw(self.win)

        #agent communication channels
        self.producer = udp.producer()
        self.consumer = udp.consumer()

        #latest sensor values, init to -1
        self.latest_sonar = -1
        self.latest_gps = [-1,-1,-1,-1]
        self.latest_energy = -1
        self.latest_network = {}





    def set_latest(self,sensor_name, sensor_value):
        """
        sets the latest reading acquired from a sensor
        """
        #TODO maybe also keep a buffer of values or something
        if hasattr(self,'latest_'+sensor_name):
            setattr(self, 'latest_'+sensor_name, sensor_value)

    def _make_debug_text(self):
        t = []
        t.append('tim;'+u.float_format2(self.time))
        t.append('snr;'+u.float_format2(self.latest_sonar))
        t.append('pos;'+u.float_format2(self.latest_gps[0])+','+
                        u.float_format2(self.latest_gps[1]))
        t.append('vel;'+u.float_format2(self.latest_gps[2])+','+
                        u.float_format2(self.latest_gps[3]))

        t.append('nrg;'+u.float_format2(self.latest_energy))
        t.append(str(self.latest_network))

        txt = ''
        for field in t:
            txt += field+'\n'
        return txt

    def _request(self,msg):
        """
        convenience method to send requests to the body
        should not be used other than simple tests!
        """
        self.producer.send(u.msg(self.id+'-body',msg))


    def control(self):
        self.time = time.time() - self.start_time
        #from the incoming queue, only read the ones addressed to this agent
        messages = u.msgs_to_self(self.addr, self.consumer)

        #if there are messages to this agent, handle'em
        if len(messages) > 0:
            print('### agent received;', messages)
            for message in messages:
                sensor_name = message['type']
                sensor_value = message['value']
                if sensor_value is not None:
                    #received some values of a sensor, set thos in current state
                    self.set_latest(sensor_name, sensor_value)


        #TODO do something with the data
        #set the debug text to sensor readings
        self.debugtext.setText(self._make_debug_text())

        #send new commands to simulation/hardware
        key = self.win.checkKey()
        if key is not None and key != '':
            if key=='Escape':
                return True
            #TODO remove this
            self._request(key)

        return False


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
