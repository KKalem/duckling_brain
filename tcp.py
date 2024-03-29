# -*- coding: utf-8 -*-
"""
Created on Wed May 10 14:39:59 2017

@author: ozer
"""
from __future__ import print_function

import config as c
import util as u

#import sys
import socket
import json
import time
#import pynmea2

#def read_tcp(s,timeout):
#    try:
#        msg = s.recv(4096)
#    except socket.timeout, e:
#        err = e.args[0]
#        # this next if/else is a bit redundant, but illustrates how the
#        # timeout exception is setup
#        if err == 'timed out':
#            time.sleep(timeout)
#            print('recv timed out, retry later')
#            return 'timeout'
#        else:
#            print(e)
#            sys.exit(1)
#    except socket.error, e:
#        # Something else happened, handle error, exit, etc.
#        print(e)
#        sys.exit(1)
#    else:
#        if len(msg) == 0:
#            print('orderly shutdown on server end')
#            sys.exit(0)
#        else:
#            return msg
#            # got a message do something :)


def package_data(data, data_type='request'):
    """
    package the data into an nmea sentence
    data type can be 'request' or 'command'
    """
    nmea = 'bad'

    if data_type == 'request':
        if data == c.GET_GPS:
            nmea = '$MSGPO,*00\r\n'
        if data == c.GET_SONAR:
            nmea = '$MSSON,*00\r\n'
    else:
        if data.find('set_target') != -1:
            #this is a set_target command
            cmd,x,y = data.split(',')
            nmea = '$MSSCP,,,'+x+','+y+',*00\r\n'

        if data.find('$MSSTO') != -1 or data.find('$MSSTA') != -1:
            nmea = data

    if nmea == 'bad':
        print('[E] BAD DATA!', data)

    jason = {'SOURCE':'OZER',
             'TIME_UNIX':int(time.time()),
             'NMEA_SENTENCES':{'NMEA1':nmea}}
    strjson = json.dumps(jason)

    #reader expects a 4 byte header before the json object with the length
    header = str(len(strjson))
    numzeros = 4 - len(header)
    #pad to 4 bytes
    if numzeros > 0:
        zeros = numzeros*'0'
        header = zeros + header

    #add header to json string
    return header+strjson


class tcpJsonNmeaClient():
    def __init__(self,ip, port, sensor_type, sensor_addr):
        #create a tcp socket, connect to server
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#        self.client.settimeout(5)
        print('[S] Connecting to '+ip+':'+str(port))
        self.client.connect((ip,int(port)))
        print('[S] Connected')


        #these are the expected responses from the server for
        #each type of sensor
        sensor_NMEAS = {'gps':'MSGPS',
                        'sonar':'MSSON',
                        'agent':'NOTHING',
                        'network':'NOTHING'}
        self.expected_NMEA = sensor_NMEAS[sensor_type]

        #for compliance
        self.sensor_addr = sensor_addr

    def send(self,data,data_type='request'):
        #obvious
        pack = package_data(data,data_type)
        self.client.send(pack)
        print('[tcp] Sent;',pack)
        self.client.send(package_data('$MSSTA,*00', data_type='command'))
        print('[tcp] ', 'Sent start')


    def receive(self,size=1024):
        #parse json
        try:
            #read the 4 bytes of header
            header = self.client.recv(4)
            json_len = int(header)
            #header contains number of chars to read
            strjson = self.client.recv(json_len)
            jason = json.loads(strjson)
        except:
            print('[E] Json parsing error!')
            print(header)
            print(strjson)
            return None

        #json has a field 'NMEA_SENTENCES'
        nmea_dict = jason.get('NMEA_SENTENCES')
        if nmea_dict is not None:
            #containing a single field, 'NMEA'
            nmea = nmea_dict.get('NMEA')

        if nmea is not None:
            #is this the type of NMEA we expect to receive?
            expected_packet = nmea.find(self.expected_NMEA)
            if expected_packet == -1:
                #this is not the packet im looking for
                return None
            else:
                #this NMEA contains the data i requested
                #parse the NMEA, its a simple comma separated list of stuff
                parts = nmea.split(',')
                #1st part is the NMEA type
                #this can be $MSGPS or $MSSON for gps or sonar readings
                nmea_type = parts[0]
                #make a packet for compliance
                data = {'to':self.sensor_addr}
                if nmea_type == '$MSGPS':
                    #heading is north=0, cw=+ from gps
                    #agent expects east=0 ccw=+  convert to that
                    heading = 360-float(parts[4])+90
                    speed = float(parts[3])

                    vx,vy = speed*u.cos(heading), speed*u.sin(heading)


                    #if not running in sim, the gps input is in lat,lon
                    #instead of meters. convert that using the same
                    #configs used to convert the polygon
                    x,y = c.LATLON_TO_XY([float(parts[1]),float(parts[2])])
                    print('lat;', parts[1], '->', x ,' lon;',parts[2],'->',y)

                    data['data'] = [x[0], #get values out of the numpy arrays
                                    y[0],
                                    vx,
                                    vy]


                if nmea_type == '$MSSON':
                    data['data'] = float(parts[2]) #depth

                #addr is unused but expected
                return 'dummy_addr',data

        return None




