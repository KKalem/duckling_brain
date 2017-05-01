# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 09:54:17 2017

@author: ozer

A general broadcast listener for debugging purposes
"""


import udp
import util as u
import time

c = udp.consumer()
while True:
    time.sleep(0.1)
    res = c.receive(size=65535)
    if res is not None:
        ip,data = res
        if data is not None and type(data)==type({1:1}):
            data = data.get('data')
            if data is not None and type(data)==type({1:1}):
                sensortype = data.get('type')
                if sensortype is not None:
                    if sensortype=='network':
                        print data.get('value')



