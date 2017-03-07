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

import sys
import traceback

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
        self.id = id
        self.addr = self.id+'-agent'
        self.win = g.GraphWin('Agent:'+str(self.id), config.CONTROL_WIN_SIZE, config.CONTROL_WIN_SIZE)
        self.win.setCoords(-config.CONTROL_WIN_SIZE/2., -config.CONTROL_WIN_SIZE/2.,
                      config.CONTROL_WIN_SIZE/2., config.CONTROL_WIN_SIZE/2.)
        self.win.setBackground('white')

        self.debugtext = g.Text(g.Point(0,0),'No-text')
        self.debugtext.draw(self.win)

        self.producer = udp.producer()
        self.consumer = udp.consumer()

    def _request(self,msg):
        self.producer.send(u.msg(self.id+'-body',msg))

    def control(self):
        #from the incoming queue, only read the ones addressed to this agent
        messages = u.msgs_to_self(self.addr, self.consumer)
        #debug
        if len(messages) > 0:
            print('data received;',messages)


        #TODO do something with the data

        #send new commands to simulation/hardware
        #TODO make a proper interface
        key = self.win.checkKey()
        if key is not None and key != '':
            if key=='Escape':
                return True
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
    else:
        print('[I] Stopped')
        agent.win.close()
