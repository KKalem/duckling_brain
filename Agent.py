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
import geometry as gm
import gp

import sys
import traceback
import time
import numpy as np
from subprocess import Popen


class Agent:
    """
    An agent that will control a body in simulation through some comm. channel
    This object does not store data that it did not receive over the comm.
    channel.
    """
    def __init__(self, id, **kwargs):
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

        #remote or auto
        #remote for getting keyboard input to control speed/turn
        self.mode = 'remote'

        #TODO devise some method of storing information from other agents
        # also look at make_measurement_list when done
        #this ties into the network stuffs
        self.other_agents = []

        #the GP object for this agent
        self.gp = gp.GP()

        #last known heading from the gps. This is useful if/when the agent is
        #not moving forward at all.
        self.heading = (-1,-1)

        #currently set target the agent should be trying to reach
        self.target = None
        #a line connecting the agent to its target, for graphics pruposes only.
        self.target_g = None

        #known max turning rate of the agent in degrees
        self.max_turn = kwargs.get('max_turn', 15.)
        #known max forward speed of the agent in m/s
        self.max_speed = kwargs.get('max_speed', 1.5)


        #the bounding polygon for this agent to stay inside
        #use a simple rectangle if none given
        #square side = 2a, centered at 0,0
        a = config.WINDOW_METERS/2.
        #safety distance to shrink
        s = 10.
        rect = [[a-s, a-s],
                [-a+s, a-s],
                [-a+s, -a+s],
                [a-s,-a+s]]
        self.bounds = kwargs.get('bounds',rect)


        ######################################################################
        # sensor stuff
        ######################################################################
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

        #a list of (x,y,d) for sensor/gps measurements.
        #x,y or d can be None or repeated if sensor data is missing for some reason
        self.measurements = []
        #a list of integer IDs for each measurement. This is used when merging
        #with other agents
        self.m_ids = []

        #last known velocity of the body (vx,vy)
        self.V = (0.,0.)

        #search radius for good points to explore
        self.start_range = config.DEFAULT_START_RANGE
        self.end_range = config.DEFAULT_END_RANGE
        self.circle_count = config.DEFAULT_CIRCLE_COUNT
        self.max_range = config.WINDOW_SIZE

        ######################################################################
        # display
        ######################################################################
        #a window to draw stuff about this agent
        self.win = g.GraphWin('Agent:'+str(self.id),
                              config.CONTROL_WIN_SIZE,
                              config.CONTROL_WIN_SIZE,
                              autoflush=False)
        self.win.setCoords(-config.CONTROL_WIN_SIZE/2., -config.CONTROL_WIN_SIZE/2.,
                      config.CONTROL_WIN_SIZE/2., config.CONTROL_WIN_SIZE/2.)
        self.win.setBackground(g.color_rgb(220,250,255))

        #some debug text to display on the control window
        self.debugtext = g.Text(g.Point(-config.CONTROL_WIN_SIZE/2.+70,0),'No-text')
        self.debugtext.draw(self.win)

        #draw the bounding polygon
        rect_g = map(lambda l: g.Point(l[0]*config.CONTROL_PPM, l[1]*config.CONTROL_PPM), self.bounds)
        self.bounds_g = g.Polygon(rect_g)
        self.bounds_g.setOutline('red')
        self.bounds_g.draw(self.win)


        ######################################################################
        # comms
        ######################################################################
        #agent communication channels
        self.producer = udp.producer()
        self.consumer = udp.consumer()


    def draw_self(self):
        """
        draws a representation of the agents own state
        """
        try:
            i = 0
            max_i = len(self.measurements)
            nones = [0]
            while i<max_i and len(nones)>0:
                i+=1
                x,y,d = self.measurements[-i]
                nones = filter(lambda v: v is None, [x,y,d])

            vx,vy = self.V
            # draw the gps values on the display
            x *= config.CONTROL_PPM
            y *= config.CONTROL_PPM
            vx *= 20.
            vy *= 20.
            l = g.Line(g.Point(x,y),g.Point(x+vx,y+vy))
            l.setArrow('last')
            l.draw(self.win)

            #draw the sonar values
            d += 10
            cir = g.Circle(g.Point(x,y), 2*config.CONTROL_PPM)
            color = g.color_rgb(255-(d*4),150, d*4)
            cir.setFill(color)
            cir.setOutline(color)
            cir.draw(self.win)

            if self.target is not None:
#                self.target_line.undraw()
                self.target_g = g.Circle(g.Point(self.target[0]*config.CONTROL_PPM,
                                                  self.target[1]*config.CONTROL_PPM),
                                         1*config.CONTROL_PPM)
                self.target_g.setFill('red')
                self.target_g.setOutline('white')
                self.target_g.draw(self.win)
        except:
            pass

    def get_latest_gps(self):
        """
        returns the latest known gps position from measurements
        """
        m = None
        i = 1
        while m is None and i < len(self.measurements):
            m = self.measurements[-i][:2]
            i += 1
        return m

    def time_to_reach(self, origin, pts):
        """
        returns approximate time to reach the given pts from origin
        """
        origin = np.array(origin)
        times = []
        for pt in pts:
            pt = np.array(pt)
            relative_pt = pt - origin
            #angle from self heading to point
            angle = gm.directed_angle(self.V, relative_pt)

            #time to turn towards point
            turn_time = np.abs(angle)/u.to_rad(self.max_turn)

            #distance to go forward
            dist = gm.euclid_distance(origin, pt)

            #time to reach point going forward
            forw_time = dist / self.max_speed

            total_time = turn_time+forw_time

            if total_time <= 0:
                print('total time 0')
                print('forw_time',forw_time,'dist',dist)
                print('turn_time',turn_time,'angle',angle)

            times.append(total_time)

        return times

    def get_enough_measurements(self, enough = 165., take_last = 20):
        #TODO return just enough measurements from self and others instead
        if len(self.measurements) > enough:
            #always take the last 20 with some skip
            skip_last = 3
            take_last *= skip_last
            res = self.measurements[-take_last::skip_last]
            skip = np.round((len(self.measurements)-len(res)) / enough)
            skip = max(1,skip)
            res.extend(self.measurements[::int(skip)])
            return res
        else:
            return self.measurements


    def generate_targets(self):
        current_pos = self.get_latest_gps()

        if current_pos is None:
            print('[E] I have no idea where I am!')
            return None

        #search for a point around the agent in a donut shape with increasing
        #inner radii.
        targets = []
        #some candidate points around the agent
        candidates = u.make_circle_targets(center=current_pos,
                                           radii = range(self.start_range ,self.end_range),
                                           count = self.circle_count)

        #filter out-of-bounds candidates
        candidates = filter(lambda p: gm.ptInPoly(self.bounds, p), candidates)

        #not a single in-bound candidate point was found. give up :(
        #or we even searched the entire map, still no points
        if candidates is None or len(candidates) < 1:
            print('[I] No candidates found!')
            return None

        #fit the gp to current measurements, this fitted model will be used later
        self.gp.fit(self.get_enough_measurements())

        #means and std devs. of candidate points given all measurements
        means, stds = self.gp.regress(candidates)

        combined = zip(candidates, means, stds)

        #prevent going to 'explored' points, collect the unexplored ones
        unexplored = filter(lambda c: c[2] > config.MIN_STD, combined)
        if unexplored is not None and len(unexplored) > 0:
            #extract the points only from the triples of (candidate, mean, std)
            target_points = zip(*unexplored)[0]
            targets.extend(target_points)

            ##### paint the map with points
            if config.PAINT_UNEXPLORED:
                for pt, mean, std in unexplored:
                        c = g.Point(pt[0]*config.CONTROL_PPM,pt[1]*config.CONTROL_PPM)
                        c.setFill('white')
                        c.setOutline('white')
                        c.draw(self.win)

        ##### paint the control map with explored areas
        if config.PAINT_EXPLORED:
            start = time.time()
            explored = filter(lambda c: c[2] <= config.MIN_STD, combined)
            if explored is not None and len(explored) > 0:
                for pt, mean, std in explored:
                    c = g.Point(pt[0]*config.CONTROL_PPM,pt[1]*config.CONTROL_PPM)
                    c.setFill('green')
                    c.setOutline('green')
                    c.draw(self.win)
            print('[I] took '+str(time.time()-start)+' seconds for painting')
        #####


        return targets


    def choose_target(self, targets):
        """
        returns the target point that is quickest to reach with the most std deviation
        """
        current_pos = self.get_latest_gps()

        #the value of a point is all the values on the path to that point
        path_stds = []
        target_stds = []
        start = time.time()
        #put all paths into a single list for GP to regress. Looping over paths
        #takes forever and is inefficient
        paths = []
        path_lens = []
        for candidate in targets:
            dist = gm.euclid_distance(current_pos, candidate)
            path = gm.subdividePath([current_pos, candidate], int(dist))
            paths.extend(path)
            path_lens.append(len(path))

        #regress for all paths at once
        all_means, all_stds = self.gp.regress(paths)

        #separate the paths again
        for i in range(len(path_lens)):
            if i == 0:
                prev = 0
                stds = all_stds[0:path_lens[i]]
            else:
                prev = path_lens[i-1]
                stds = all_stds[prev + 1 : prev + path_lens[i]]

            path_stds.append(sum(stds))
            target_stds.append(stds[-1]) #last one is the candidate always
        print('[I] took '+str(time.time()-start)+' seconds for path stds')

        #time to reach these candidate points
        times = self.time_to_reach(current_pos, targets)

        times = np.array(times)
        path_stds = np.array(path_stds)
        target_stds = np.array(target_stds)

        #value of the candidates. quickest most deviating point please
        values = path_stds / times
        best_point = np.argmax(values)

#        print('[I] from',current_pos,
#                '\n[I] chosen std path',path_stds[best_point],
#                '\n[I] chosen std target',target_stds[best_point])

        return targets[best_point]


    def try_setting_target(self):
        """
        tries to find and choose a target once. If fails, increases radius and returns
        without re-trying with the new radius. If succeeds, resets the radii and
        sets the target.
        """
        #dont move while calculating a target
        self.send_to_body('set_stop')
        #generate targets around the agent
        targets = self.generate_targets()
        #do we have enough targets to make a decision?
        if targets is not None and len(targets) >= config.MIN_UNEXPLORED:
            #we have enough targets, choose one and set it
            self.target = self.choose_target(targets)
            #reset the radii in case they were increased for this particular search
            self.start_range = config.DEFAULT_START_RANGE
            self.end_range = config.DEFAULT_END_RANGE
            self.circle_count = config.DEFAULT_CIRCLE_COUNT
            self.max_range = config.WINDOW_SIZE
            return True
        else:
            #we dont have enough targets, increase search radius and pass
            print('[I] Increasing search radius from',self.end_range)
            self.start_range = self.end_range
            self.end_range += config.SEARCH_INCREMENT
            self.circle_count *= 2
            return False





###############################################################################
# actual control that is run continuesly
###############################################################################
    def control(self):
        #time since starting control
        self.time = time.time() - self.start_time

        #draw the agent on display
        self.draw_self()

        ######################################################################
        # sensor readings
        ######################################################################
        #from the incoming queue, only read the ones addressed to this agent
        messages = u.msgs_to_self(self.addr, self.consumer)
        #if there are messages to this agent, handle'em
        if len(messages) > 0:
            for message in messages:
                sensor_type = message.get('type')
                sensor_value = message.get('value')
                if sensor_value is not None and sensor_type is not None:
                    if sensor_type == 'network':
                        #TODO handle network readings
                        pass
                    if sensor_type == 'gps':
                        #velocity and heading from gps
                        self.V = sensor_value[2:]
                        speed = gm.euclid_distance([0,0],self.V)
                        if speed > 0.1:
                            self.heading = self.V

                        #position from gps
                        x,y = sensor_value[:2]
                        #do we have a previous measurement?
                        if len(self.measurements) > 0:
                            old_x, old_y, _ = self.measurements[-1]
                            #does this measurement have a position added?
                            if old_x is None or old_y is None:
                                #it needs a position, add it
                                self.measurements[-1][0] = x
                                self.measurements[-1][1] = y
                            else:
                                #it doesnt need a position
                                #is the new point far enough away?
                                distance = gm.euclid_distance([old_x,old_y],[x,y])
                                if distance >= config.DISTANCE_THRESHOLD:
                                    self.measurements.append([x,y,None])
                                    new_id = self.m_ids[-1] +1
                                    self.m_ids.append(new_id)
                        else:
                            #no previous measurements, this is the first
                            self.measurements.append([x,y,None])
                            self.m_ids.append(0)

                    if sensor_type == 'sonar':
                        d = sensor_value
                        #any previous measurements?
                        if len(self.measurements) > 0:
                            #does the last added gps need a depth?
                            _, _, old_d = self.measurements[-1]
                            if old_d is None:
                                #its missing depth, add it
                                self.measurements[-1][2] = d


        #set the debug text to sensor readings
        self.debugtext.setText(self._make_debug_text())


        ######################################################################
        # controls
        ######################################################################
        key = self.win.checkKey()
        #end control
        if key=='Escape':
            return True
        #switch between remote and autonomous controls
        if key == 'a':
            self.mode = 'auto'
        if key == 'r':
            self.mode = 'remote'
        if key == 't':
            #save the traces
            self.save_trace()
        if key == 'g':
            #draw the GP regressed surface
            try:
                self.gp.show_surface(self.measurements)
            except:
                print('[E] no surface to show yet')

        #mouse controls
        mouse = self.win.checkMouse()
        if mouse is not None:
                x = mouse.getX()/config.CONTROL_PPM
                y = mouse.getY()/config.CONTROL_PPM
                print('mouse_meters;',u.float_format2(x),u.float_format2(y))

                #prevent interfering when in auto-mode
                if self.mode == 'remote':
                    self.target = (x,y)
                    self.send_to_body('set_target'+','+str(x)+','+str(y))

        #send new commands to simulation/hardware
        #remote control
        if self.mode == 'remote':
            #the sent strings should have same-named methods with no arguments
            #in body. def inc_speed(self): .....
            if key=='Up':
                self.send_to_body('inc_speed')
            if key=='Down':
                self.send_to_body('dec_speed')
            if key=='s':
                self.send_to_body('set_stop')
            if key=='w':
                self.send_to_body('set_max_speed')
            if key=='Left':
                self.send_to_body('turn_left')
            if key=='Right':
                self.send_to_body('turn_right')
            if key=='x':
                self.send_to_body('reset_turn')

            if key=='q':
                self.target = self.choose_target()
                self.set_body_target(self.target)

###############################################################################
# AUTO CONTROL
###############################################################################
        if self.mode == 'auto':
            #autonomous control
            #got a target?
            if self.target is not None:
                #got a target, we should be moving towards it, or already reached
                #reached target?
                dist = gm.euclid_distance(self.get_latest_gps(), self.target)
                if dist < config.TARGET_DISTANCE_THRESHOLD + 2:
                    #we have a target and we reached it.
                    target_set = self.try_setting_target()
                    if target_set:
                        self.set_body_target(self.target)
                else:
                    #we have a target but not reached yet. Wait to reach it
                    pass
            else:
                #we dont have a target
                #do we have enough measurements to even choose a target?
                m_len = len(self.measurements)
                if m_len >= config.MEASUREMENT_COUNT_THRESHOLD:
                    #we do have enough measurements
                    target_set = self.try_setting_target()
                    if target_set:
                        self.set_body_target(self.target)
                else:
                    #we dont even have enough measurements to regress properly
                    #move the body forward until we do have enough measurements
                    print('[I] Moving forward a little to get some measurements')
                    self.send_to_body('set_max_speed')

        g.update(config.UPDATE_FPS)
        return False

    def set_body_target(self, target):
        x,y = target
        self.send_to_body('set_target'+','+str(x)+','+str(y))
        print('[T] set target to'+ str(self.target))


    def send_to_body(self,msg):
        """
        convenience method to send requests to the body
        should be used to control the body of the agent
        """
        self.producer.send(u.msg(self.id+'-body',msg))


    def save_trace(self):
        """
        save the gps and sonar measurements to a file for reasons
        """
        with open(config.TRACE_DIR+self.id+'_trace','w') as f:
            for x,y,d in self.measurements:
                f.write(str(x)+','+str(y)+','+str(d)+'\n')
        print('[I] trace saved to; '+config.TRACE_DIR+self.id+'_trace')

    def _make_debug_text(self):
        t = '-'
        try:
            i = 0
            max_i = len(self.measurements)
            nones = [0]
            while i<max_i and len(nones)>0:
                i+=1
                x,y,d = self.measurements[-i]
                nones = filter(lambda v: v is None, [x,y,d])

            vx,vy = self.V
            t = ''
            t += 'x,y,d; '+u.float_format2(x)+','+u.float_format2(y)+','+u.float_format2(d)+'\n'
            t += 'vx,vy; '+u.float_format2(vx)+','+u.float_format2(vy)+'\n'
            t += 'ms; '+str(len(self.measurements))+'\n'
        except:
            return t
        return t


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
