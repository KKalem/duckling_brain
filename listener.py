# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 09:54:17 2017

@author: ozer

A general broadcast listener for debugging purposes
"""


import udp
import util as u

c = udp.consumer()
while True:
    messages = u.msgs_to_self('0',c)
    if len(messages) > 0:
        for message in messages:
            sensor_type = message.get('type')
            sensor_value = message.get('value')
            if sensor_value is not None and sensor_type is not None:
                if sensor_type == 'network':
                    print sensor_value



