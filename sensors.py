# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 22:49:58 2017

@author: ozer
"""
from __future__ import print_function

import udp
import config
import util as u

import sys
import traceback
import time

class PureSensor:
    """
    A general sensor class
    Blocks while waiting for a response
    Relays the relevant info to its agent
    Requests data all the time
    """
    def __init__(self, sensor_type, id, request_sentence, poll_rate):
        #the sensor will receive readings from the body
        self.body_addr = id+'-body'
        #and send the readings when they arrive to the agent
        self.agent_addr = id+'-agent'
        #this sentence will be used to request the reading from the body
        self.request_sentence = request_sentence
        #this is the address of this sensor, it will look for a message sent
        #to this address and relay that to the agent then stop
        self.addr = id+'-'+sensor_type

        #use this sensor type to report to agent
        self.sensor_type = sensor_type

        #limit the polling rate
        self.poll_rate = poll_rate
        self.last_poll = time.time()
        self.first_poll = True

        #comms channel with body and agent
        self.p = udp.producer()
        self.c = udp.consumer(blocking=0)

    def run(self):
        """
        Requests and retrieves the relevant sensory data.
        Relays the data to the agent
        Requests from hardware/sim
        """
        #The sensor runs all the time, but doesn't busy wait
        while True:
            #TODO make polling rate work
            #check if we even can do anything
            dt = time.time() - self.last_poll

            #sleep to fulfill the polling rate
            if dt > self.poll_rate or self.first_poll:
                #we can poll now, gogo
                #request the relevant data
                self.p.send(u.msg(self.body_addr,self.request_sentence))
                self.last_poll = time.time()
                print('### polled '+self.sensor_type)
                self.first_poll = False
            else:
                #can't poll yet, sleep a little
                print('### sleeping '+self.sensor_type)
                time.sleep(0.05)
                print('### woke '+self.sensor_type)

            #check if we got any responses
            response = None
            #check all responses over the pipe
            while response is None:
                #receive some packet
                packet = self.c.receive()
                #did we receive anything?
                if packet is not None:
                    addr, data = packet
                    #is packet meant for this sensor?
                    if data['to'] == self.addr:
                        #we got the response we needed
                        response = data['data']
                        print('### response:',response)
                else:
                #wait for another packet
                    break
            if response is not None:
                #we got the response we expected, relay it to the agent
                to_agent = {'type':self.sensor_type, 'value':response}
                self.p.send(u.msg(self.agent_addr, to_agent))
                print('### sent to agent;',to_agent)


def make_spawn_command(id, sensor_name, request_sentence, poll_rate):
    """
    convenince function to generate the command string for pexpect to run
    """
    return ['python',
            'sensors.py',
            str(id),
            str(sensor_name),
            str(request_sentence),
            str(poll_rate)]

if __name__=='__main__':
    try:
        #TODO properly parse these etc
        id = sys.argv[1]
        type = sys.argv[2]
        sentence = sys.argv[3]
        poll_rate = float(sys.argv[4])
    except:
        #just for debugging
        id = '0'
        type = 'gps'
        sentence = 'get_gps'
        poll_rate = 0.1

    try:
        print('[I] Running '+type)
        sensor = PureSensor(type, id, sentence, poll_rate)
        sensor.run()
    except:
        traceback.print_exc(file=sys.stderr)
        print('[I] Stopped '+type)
    else:
        print('[I] Stopped '+type)





