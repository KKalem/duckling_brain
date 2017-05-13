# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 22:49:58 2017

@author: ozer
"""
from __future__ import print_function

import udp
import tcp
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
    def __init__(self,
                 sensor_type,
                 id,
                 request_sentence,
                 poll_rate,
                 packet_size=1024,
                 ip = None,
                 port = None):
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

        #packet size the udp listener will read
        #set to large (max 64k) for large expected packets
        self.packet_size = packet_size

        #comms channel with body and agent
        self.p = udp.producer()
        self.c = udp.consumer(blocking=0)

        if not config.SIMULATION:
            self.tcp = tcp.tcpJsonNmeaClient(ip,
                                             int(port),
                                             self.sensor_type,
                                             self.addr)

    def run(self):
        """
        Requests and retrieves the relevant sensory data.
        Relays the data to the agent
        Requests from hardware/sim
        """
        #The sensor runs all the time, but doesn't busy wait
        while True:
            #check if we even can do anything
            dt = time.time() - self.last_poll

            #sleep to fulfill the polling rate
            if dt > self.poll_rate or self.first_poll:
                #we can poll now, gogo
                #request the relevant data
                if config.SIMULATION:
                    self.p.send(u.msg(self.body_addr,self.request_sentence))
                else:
                    #dont do nothin, the body is emitting on a regular basis
                    pass
#                    self.tcp.send(self.request_sentence)
                self.last_poll = time.time()
                self.first_poll = False
            else:
                #can't poll yet, sleep a little
                time.sleep(0.005)

            #check if we got any responses
            response = None
            #check all responses over the pipe
            while response is None:
                #receive some packet
                if config.SIMULATION:
                    packet = self.c.receive(size=self.packet_size)
                else:
                    packet = self.tcp.receive(size=self.packet_size)
                #did we receive anything?
                if packet is not None:
                    addr, data = packet
                    #is packet meant for this sensor?
                    if data['to'] == self.addr:
                        #we got the response we needed
                        response = data['data']
                    if self.sensor_type == 'network':
                        #if this is a network sensor, listen to broadcasts
                        if data['to'] == 'broadcast':
                            response = data['data']
                else:
                #wait for another packet
                    break
            if response is not None:
                #we got the response we expected, relay it to the agent
                to_agent = {'type':self.sensor_type, 'value':response}
                self.p.send(u.msg(self.agent_addr, to_agent))


def make_spawn_command(id,
                       sensor_type,
                       request_sentence,
                       poll_rate,
                       packet_size=1024,
                       ip='0.0.0.0',
                       port=12):
    """
    convenince function to generate the command string for pexpect to run
    """
    return ['python',
            'sensors.py',
            str(id),
            str(sensor_type),
            str(request_sentence),
            str(poll_rate),
            str(packet_size),
            str(ip),
            str(port)]

if __name__=='__main__':
    try:
        id = sys.argv[1]
        type = sys.argv[2]
        sentence = sys.argv[3]
        poll_rate = float(sys.argv[4])
        packet_size=int(sys.argv[5])
        ip = sys.argv[6]
        port = int(sys.argv[7])
    except:
        #just for debugging
        id = '0'
        type = 'gps'
        sentence = 'get_gps'
        poll_rate = 0.1
        packet_size=1024
        ip = '10.13.20.23'
        port = 12

    try:
        print('[I] Running '+type)
        sensor = PureSensor(type, id, sentence, poll_rate, packet_size, ip=ip, port=port)
        sensor.run()
    except:
        traceback.print_exc(file=sys.stderr)
        print('[I] Stopped '+type)
    else:
        print('[I] Stopped '+type)





