# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 19:35:31 2017

@author: https://www.raspberrypi.org/forums/viewtopic.php?f=32&t=39431
"""

import socket
import time
import pickle

HOST_IP = "0.0.0.0" # all interfaces
SENDER_PORT = 1501
# 224.0.1.0 thru 224.255.255.255
# (ping 224.0.0.1 for the group mulitcast server list)
MCAST_ADDR = "224.168.2.9"
MCAST_PORT = 1600
TTL=31# valid value are 1-255, <32 is local network

class producer:
    def __init__(self, sender_ip=HOST_IP, sender_port=SENDER_PORT, ttl=TTL):
        try:
            self.sock = socket.socket(socket.AF_INET,
                                      socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self.sock.bind((sender_ip,sender_port))
            self.sock.setsockopt(socket.IPPROTO_IP,
                                 socket.IP_MULTICAST_TTL, ttl)
        except socket.error, e:
            if socket.error == 10048:
                self.__init__(sender_ip,sender_port+1,ttl)

    def send(self, msg="", mcast_addr=MCAST_ADDR, mcast_port=MCAST_PORT,):
        pickled_msg = pickle.dumps(msg)
        self.sock.sendto(pickled_msg, (mcast_addr, mcast_port))

    def host_name(self):
        return socket.gethostname()

class consumer:
    def __init__(self, client_ip=HOST_IP, mcast_addr=MCAST_ADDR,
                 mcast_port=MCAST_PORT, ttl=TTL, blocking=0):
        self.sock = socket.socket(socket.AF_INET,
                                  socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.bind((client_ip, mcast_port))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             socket.inet_aton(mcast_addr) + socket.inet_aton(client_ip))
        self.sock.setblocking(blocking)

    def receive(self, size=1024,):
        try:
            pickled_data, addr = self.sock.recvfrom(size)
            data = pickle.loads(pickled_data)
            return (addr,data)
        except socket.error, e:
            return None

    def host_name(self):
        return socket.gethostname()

"""
import udp
>>> # ok we create a producer using the default keyword arguments
>>> producer = udp.producer()
>>> # now lets make a client , using the default keyword arguments, so we can get the data
>>> consumer = udp.consumer()

>>> producer.send("Hello World!")
>>> # message sent to all clients
>>> # now when we want to get the message
>>> consumer.receive()
(('10.20.40.126', 1501), 'Hello World!')
>>> # what happens if we call again
>>> consumer.receive()
>>>
>>> # we get a None
>>> # You can put it in a loop and just call the receive until you get some action.
>>> # You can have multiple clients listening for the producer and each will be independant
>>> consumer2 = udp.consumer()
>>> producer.send([1,2,3,4,5])
>>> consumer.receive()
(('10.20.40.126', 1501), [1, 2, 3, 4, 5])
>>> consumer2 .receive()
(('10.20.40.126', 1501), [1, 2, 3, 4, 5])
>>> # you may notice the ip and port at the start of the message you can just ignore it for now
>>> msg = consumer2 .receive()
>>> if msg:
            ip_port, data = msg
"""