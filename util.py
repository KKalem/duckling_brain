# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 19:13:19 2017

@author: ozer
bunch of utility functions shared across just about everything
"""
import numpy as np

def msgs_to_self(addr, consumer):
    """
    returns a list of messages received by the given consumer, addressed to 'id'
    """
    all_received = []
    packet = consumer.receive()
    while packet is not None:
        ip_addr, data = packet
        if data['to'] == addr:
            all_received.append(data['data'])
        packet = consumer.receive()
#    print('msgs_to_self',addr,all_received)
    return all_received

def msg(to,data):
    """
    Just a simple function to make simple messages
    """
    return {'to':to,'data':data}

def clamp(minval,maxval, val):
    """
    Simply clamps the value into min-max
    """
    return max(minval, min(maxval, val))


def to_rad(deg):
    """
    Simple degree to radian conversion
    """
    return np.pi*deg/180.


def sin(deg):
    """
    wrapper for numpy's sin
    """
    return np.sin(to_rad(deg))


def cos(deg):
    """
    wrapper for numpy's cos
    """
    return np.cos(to_rad(deg))