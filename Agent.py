#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 21:35:28 2017

@author: ozer
"""
from __future__ import print_function

import udp
import tcp
import config
import util as u
import sensors
import geometry as gm
import gp

import sys
import traceback
import time
import numpy as np
from subprocess import Popen

if not config.HEADLESS:
    import graphics as g
    import matplotlib.pyplot as plt

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

        #ip:port of the physical body if available
        self.body_ip = kwargs.get('body_ip')
        self.body_port = kwargs.get('body_port')

        #keep track of agent time since conception
        self.time = 0.
        self.start_time = time.time()

        #remote or auto
        #remote for getting keyboard input to control speed/turn
        self.mode = 'remote'

        #number of samples to use when fitting GP.
        #this is increased near the end when no 'unexlpored' points
        #could be found. resets after finding a target
        self.enough_points = kwargs.get('enough_points',config.DEFAULT_ENOUGH)

        #velocity obstacle radii
        self.vo_r = kwargs.get('vo_r',config.VO_SELF_R)
        self.other_r = kwargs.get('other_r',config.VO_OTHER_R)

        #a map of agent_id -> properties for each of the other agents
        #this agent has encountered.
        #the properties are:
        #'id' obv.
        #'mments' a map of {m_id:mment}
        #'target' [point,value] for the current target of this other agent
        self.other_agents = {}

        #the GP object for this agent
        self.gp = gp.GP()

        #last known heading from the gps. This is useful if/when the agent is
        #not moving forward at all.
        self.heading = (-1,-1)

        #currently set target the agent should be trying to reach
        self.target = None
        self.target_value = -1
        #a line connecting the agent to its target, for graphics pruposes only.
        self.target_g = None

        #known max turning rate of the agent in degrees
        self.max_turn = kwargs.get('max_turn', config.AGENT_MAX_W)
        #known max forward speed of the agent in m/s
        self.max_speed = kwargs.get('max_speed', config.AGENT_MAX_V)


        #the bounding polygon for this agent to stay inside
        #ccw point-list
        self.bounds = kwargs.get('bounds',config.POLY)

        if not config.SIMULATION:
            #map the latlon coordiantes to local coordinates
            self.bounds = map(config.LATLON_TO_XY, self.bounds)

        #generate a lattice of points inside the given polygon for later use
        #the matrix should encompass the entire given polygonal area
        self.inside_points = []
        xmax,ymax = np.max(self.bounds,axis=0)
        xmin,ymin = np.min(self.bounds,axis=0)

        xs = np.linspace(xmin,xmax,config.T_DENSITY)
        ys = np.linspace(xmin,xmax,config.T_DENSITY)
        for x in xs:
            for y in ys:
                if gm.ptInPoly(self.bounds, [x,y]):
                    self.inside_points.append([x,y])


        ######################################################################
        # sensor stuff
        ######################################################################
        #spawn commands for the sensor processes
        #default packet size is 1024 bytes and is enough for everything
        # but the network

        spawn_commands = [
        sensors.make_spawn_command(self.id,
                                   'gps',
                                   config.GET_GPS,
                                   config.GPS_POLL,
                                   ip = self.body_ip,
                                   port = self.body_port),
#        sensors.make_spawn_command(self.id,
#                                   'energy',
#                                   config.GET_ENERGY,
#                                   config.NRG_POLL,
#                                   ip = self.body_ip,
#                                   port = self.body_port),
        sensors.make_spawn_command(self.id,
                                   'sonar',
                                   config.GET_SONAR,
                                   config.SNR_POLL,
                                   ip = self.body_ip,
                                   port = self.body_port),
        sensors.make_spawn_command(self.id,
                                   'network',
                                   config.GET_NETWORK,
                                   config.NET_POLL,
                                   packet_size=4096,
                                   ip = self.body_ip,
                                   port = self.body_port)
        ]

        if kwargs.get('procs',True):
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
        self.max_range = gm.euclid_distance([xmax,ymax],[xmin,ymin])

        ######################################################################
        # display
        ######################################################################
        #a window to draw stuff about this agent
        if not config.HEADLESS:
            self.win = g.GraphWin('Agent:'+str(self.id),
                                  config.CONTROL_WIN_SIZE,
                                  config.CONTROL_WIN_SIZE,
                                  autoflush=False)


            self.win.setCoords(-config.CONTROL_WIN_SIZE/2., -config.CONTROL_WIN_SIZE/2.,
                          config.CONTROL_WIN_SIZE/2., config.CONTROL_WIN_SIZE/2.)
            self.win.setBackground(g.color_rgb(220,250,255))

            #a canvas to draw stuff on
            self.bg = g.Image(g.Point(0,0),config.CONTROL_WIN_SIZE,config.CONTROL_WIN_SIZE)


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

        #time of last broadcasts
        self.last_missing_broadcast_time = 0
        self.last_mment_broadcast_time = 0

        #if using physical body, also need a tcp client to talk to it
        #everything except ip:port is irrelevant, body should not talk to
        #the agent directly, ever. This is purely for sending commands.
        if not config.SIMULATION:
            self.tcp = tcp.tcpJsonNmeaClient(self.body_ip,
                                             self.body_port,
                                             sensor_type = 'agent',
                                             sensor_addr = self.addr)




        ######################################################################
        # TAHIROVIC-ONLY
        ######################################################################
        if config.USE_PURE_TVIC:
            self.tvic_radius = kwargs.get('tvic_radius',config.TVIC_RADIUS)

            tvic_size = config.T_DENSITY
            #'explored' = 1 unexplored = 0. this is the entire world
            tvic_matrix = np.zeros([tvic_size, tvic_size])
            #mark outside points as explored form the start
            for x in range(tvic_size):
                for y in range(tvic_size):
                    #scale tvic space to real space where the poly is defined
                    xx,yy = u.scale_range([x,y],
                                   new_min = -config.WINDOW_METERS/2.,
                                   new_max = config.WINDOW_METERS/2.,
                                   org_min = 0,
                                   org_max = tvic_size)
                    inside = gm.ptInPoly(self.bounds, [xx,yy])
                    if not inside:
                        tvic_matrix[x,y] = 1

            self.tvic_matrix = tvic_matrix

            self.to_tvic = lambda points: u.scale_range(points,
                                               new_min = 0,
                                               new_max = self.tvic_matrix.shape[0],
                                               org_min = -config.WINDOW_METERS/2.,
                                               org_max = config.WINDOW_METERS/2.)

            self.to_real = lambda points: u.scale_range(points,
                                               new_min = -config.WINDOW_METERS/2.,
                                               new_max = config.WINDOW_METERS/2.,
                                               org_min = 0,
                                               org_max = self.tvic_matrix.shape[0])



    def draw_self(self):
        """
        draws a representation of the agents own state
        """
        if config.HEADLESS:
            return
        i = 0
        max_i = len(self.measurements)
        nones = [0]

        x,y,d = (None,None,None)
        while i<max_i and len(nones)>0:
            i+=1
            x,y,d = self.measurements[-i]
            nones = filter(lambda v: v is None, [x,y,d])


        vx,vy = self.V

        if x is None or y is None or d is None:
            return
        # draw the gps values on the display
        x *= config.CONTROL_PPM
        y *= config.CONTROL_PPM
        vx *= config.CONTROL_PPM
        vy *= config.CONTROL_PPM

        #draw the sonar values
        d += 10
        cir = g.Circle(g.Point(x,y), 2*config.CONTROL_PPM)
        color = g.color_rgb(255-(d*4),150, d*4)
        cir.setFill(color)
        cir.setOutline('red')
        cir.draw(self.win)

        #draw an arrow for velocity
        l = g.Line(g.Point(x,y),g.Point(x+vx,y+vy))
        l.setArrow('last')
        l.draw(self.win)

        if self.target is not None:
            self.target_g = g.Circle(g.Point(self.target[0]*config.CONTROL_PPM,
                                              self.target[1]*config.CONTROL_PPM),
                                     1*config.CONTROL_PPM)
            self.target_g.setFill('red')
            self.target_g.setOutline('white')
            self.target_g.draw(self.win)


    def draw_others(self):
        if config.HEADLESS:
            return
        for other_id, data in self.other_agents.iteritems():
            mments = data['mments']
            target = data['target']
            last_mment_key = max(mments.keys())
            last_mment = mments[last_mment_key]
            x,y = last_mment[:2]
            vx,vy = last_mment[2:4]
            d = last_mment[4]
            # draw the gps values on the display
            x *= config.CONTROL_PPM
            y *= config.CONTROL_PPM
            vx *= config.CONTROL_PPM
            vy *= config.CONTROL_PPM


            #draw the sonar values
            d += 10
            cir = g.Circle(g.Point(x,y), 2*config.CONTROL_PPM)
            color = g.color_rgb((255-(d*4)),150., (d*4))
            cir.setFill(color)
            cir.setOutline(color)
            cir.draw(self.win)

            if vx>=0 and vy>=0:
                l = g.Line(g.Point(x,y),g.Point(x+vx,y+vy))
                l.setFill('gray')
                l.draw(self.win)

            #draw the target
            if target is not None:
                pos,val = target
                if pos is not None:
                    x,y = pos
                    x *= config.CONTROL_PPM
                    y *= config.CONTROL_PPM
                    cir = g.Circle(g.Point(x,y), 0.5*config.CONTROL_PPM)
                    cir.setFill('green')
                    cir.setOutline('black')
                    cir.draw(self.win)

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

    def get_enough_measurements(self, take_last = 20, skip_last = 3):
        """
        returns enough measurements for GP to get an 'ok' regression in
        limited time.
        take_last = # of points to always take from the last n measurements
        skip_last = how many points to skip in the take_last. this is to keep the
        most recent trai of the agent always accurate and unchanging with number of points
        in the rest of the map.
        """
        #make a list of all measurements, sorted by time=id
        #for each m-list, take the last N items, then take 'enough' from the
        #rest, combined for all agents. Result should be a homogeneous selection
        #of points across all agents.

        #use the agent-defined number of points as target
        enough = self.enough_points

        #number of points all agents, including self have
        total_points = len(self.measurements)

        #measurement maps of other agents
        other_mments = []
        other_ids = []
        for other_agent_map in self.other_agents.itervalues():
            mments = other_agent_map['mments']
            other_ids.append(mments.keys())
            mments = map(lambda m: (m[0],m[1],m[4]), mments.values()) #x,y,vx,vy,d
            other_mments.append(mments)
            total_points += len(mments)

        #if there is only enough points, no need to thin them down.
        #give it some buffer for the rare missing points from other agents
        if total_points <= enough+10:
            ret = []
            ret.extend(self.measurements)
            for other in other_mments:
                ret.extend(other)
            return ret

        #there be more than enough measurements, gotta thin them down
        #for easy slicing
        take_last *= skip_last
        #number of taken points from the last parts
        last_taken = 0
        #last points from self measurements, easy with an already sorted array
        if len(self.measurements) <= take_last:
            self_last = self.measurements
            self_remaining = 0
        else:
            self_last = self.measurements[-take_last::skip_last]
            self_remaining = len(self.measurements) - len(self_last)
        last_taken += len(self_last)
        #last points from other agents
        other_last = []
        #sorted-by-id mments of other agents. all measurements in this.
        other_sorted = []
        #remaining number of points from each agent
        other_remaining = []
        for mments,ids in zip(other_mments, other_ids):
            #does this particular agent have enough to thin down?
            if len(ids) <= take_last:
                #it does not. so just take all available points
                other_last.append(mments)
                last_taken+=len(mments)
                other_remaining.append(0)
            else:
                #it does have more than the last points
                #take the required last points, then get whats left later
                zipped = zip(ids,mments)
                sorted_by_id = sorted(zipped, key=lambda e: e[0]) #sort by id
                sorted_mments = zip(*sorted_by_id)[1] #separate mments from ids
                other_sorted.append(sorted_mments) #keep for later
                taken = sorted_mments[-take_last::skip_last]
                other_last.append(taken)
                last_taken += len(taken)
                other_remaining.append(len(sorted_mments) - len(taken))


        #now that the last points are taken, how many points are left to take from
        #the tails?
        tail_count = total_points - last_taken
        #how many points to take in total from the tails
        remaining = enough - last_taken
        #to reach this number of points, how many should be skipped from each mment list
        tail_skip = int(np.round(tail_count / float(remaining)))

        #does this agent have any more to take?
        if self_remaining <= 0:
            #nope, we already took everything this agent had
            self_tail = []
        else:
            #its got a tail! cut it, it'll grow back.
            self_tail = self.measurements[:-take_last:tail_skip]

        #tails of other agents
        other_tail = []

        #do the same for other agent measurements
        for mments,remaining in zip(other_sorted,other_remaining):
            if remaining <= 0:
                other_tail.append([])
            else:
                tail = mments[:-take_last:tail_skip]
                other_tail.append(tail)

        #now we have the heads and tails of all others and self
        ret = []
        ret.extend(self_last)
        ret.extend(self_tail)
        for other_tail, other_last in zip(other_tail, other_last):
            ret.extend(other_last)
            ret.extend(other_tail)
        return ret

    def scale_means(self, means):
        """
        scales the given means[0..1] into currently known range of real depth values
        """
        #make a list of all known depth values. THis will be used to scale
        #the [0..1] means to depths
        all_depths = []
        #depths gathered by this agent
        self_depths = map(lambda m: m[2], self.measurements)
        self_depths = filter(lambda m: m is not None, self_depths)
        all_depths.extend(self_depths)
        #depths gathered by other agents
        for other_agent in self.other_agents.itervalues():
            mments = other_agent['mments']
            mments = map(lambda m: m[2], mments.values())
            mments = filter(lambda m: m is not None, mments)
            all_depths.extend(mments)

        #scale the means to depths
        scaled_means = u.scale_range(means, min(all_depths), max(all_depths))
        return scaled_means


    def filter_land(self, means, points, stds):
        """
        given all the measurements we know, and a fitted GP. Use the means
        to estimate which of the given points lie over water and return
        the ones that are NOT over surface
        """
        if not config.AVOID_LAND:
            return means,points,stds

        scaled_means = self.scale_means(means)

        #filter the ones that are deep enough
        filtered = filter(lambda p: p[0] > config.DEPTH_LIMIT, zip(scaled_means, points, stds))

        if config.PAINT_LAND:
            if filtered is not None and len(filtered) > 0:
                _,waters,__ = zip(*filtered)
                for water in waters:
                    try:
                        rect = g.Circle(g.Point(water[0]*config.CONTROL_PPM,
                                                water[1]*config.CONTROL_PPM), 5)
                        rect.setFill('blue')
                        rect.setOutline('blue')
                        rect.draw(self.win)
                    except:
                        print('problem water',water)

            #land parts for painting
            lands = filter(lambda p: p[0] <= config.DEPTH_LIMIT, zip(scaled_means, points, stds))
            if lands is not None and len(lands) > 0:
                _,lands,__ = zip(*lands)
                for land in lands:
                    try:
                        rect = g.Circle(g.Point(land[0]*config.CONTROL_PPM,
                                                land[1]*config.CONTROL_PPM), 5)
                        rect.setFill('black')
                        rect.setOutline('black')
                        rect.draw(self.win)
                    except:
                        print('problem land',land)

        #unzip the means from points, return points only
        scaled_means,result,stds = zip(*filtered)

        return scaled_means,result,stds



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
        #re-make the gp object. band-aid trial. THIS FKIN WORKS, FFS SCIKIT.
#        self.gp = gp.GP()
        self.gp.fit(self.get_enough_measurements(), ID=self.id, save_surface=True)

        #means and std devs. of candidate points given all measurements
        means, stds = self.gp.regress(candidates)

        #prevent going to 'explored' points, collect the unexplored ones
        combined = zip(candidates, means, stds)
        unexplored = filter(lambda c: c[2] > config.MIN_STD, combined)

        if unexplored is not None and len(unexplored) > 0:
            #extract the points only from the triples of (candidate, mean, std)
            unexplored, means, stds = zip(*unexplored)

            if config.AVOID_LAND:
                #filter out predicted lands
                scaled_means, unexplored, stds = self.filter_land(means, unexplored, stds)

            targets.extend(unexplored)

            ##### paint the map with points
            if config.PAINT_UNEXPLORED:
                for pt in unexplored:
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
        if config.USE_TAHIROVIC:
            #instead of following the variance from self, follow high variance point
            #w/ respect to 'unexplored mean' point
            #to find the mean of unexplored land, we must regress for the entire
            #map and filter explored areas.

            #use the std of each point as a weight
            means, stds = self.gp.regress(self.inside_points)
            #filter the unexplored points
            unexplored = filter(lambda c: c[1] > config.MIN_STD, zip(self.inside_points, stds, means))
            unexplored_pts, unexplored_stds, unexplored_means = zip(*unexplored)

            if config.AVOID_LAND:
                #filter out land
                scaled_means, unexplored_pts, unexplored_stds = self.filter_land(unexplored_means,
                                                                                 unexplored_pts,
                                                                                 unexplored_stds)

            if config.PAINT_TAHIROVIC:
                #normalize means for painting
                depth_colors = u.scale_range(means, 0,255)
                for p,depth_color in zip(self.inside_points,depth_colors):
                        center = g.Point(p[0]*config.CONTROL_PPM,p[1]*config.CONTROL_PPM)
                        c = g.Circle(center,3)
                        color = g.color_rgb(30,200, depth_color)
                        c.setFill(color)
                        c.setOutline(color)
                        c.draw(self.win)

                std_colors = u.scale_range(unexplored_stds, 0,255)
                for p,std_color in zip(unexplored_pts,std_colors):
                        center = g.Point(p[0]*config.CONTROL_PPM,p[1]*config.CONTROL_PPM)
                        c = g.Circle(center,3)
                        color = g.color_rgb(140,std_color,30)
                        c.setFill(color)
                        c.setOutline(color)
                        c.draw(self.win)

            if config.T_WEIGHTED:
                pts_stds = map(lambda c: (c[0][0],c[0][1],c[1]), zip(unexplored_pts, unexplored_stds))
                pts_stds = np.array(pts_stds)
                xcentroid = np.sum(pts_stds[:,0]*pts_stds[:,2]) / np.sum(pts_stds[:,2])
                ycentroid = np.sum(pts_stds[:,1]*pts_stds[:,2]) / np.sum(pts_stds[:,2])
            else:
                xcentroid,ycentroid = np.mean(unexplored_pts,axis=0)

            #use the centroid for path calculations
            path_root = [xcentroid,ycentroid]

            if config.T_DRAW_CENTROID:
                center = g.Point(xcentroid*config.CONTROL_PPM,ycentroid*config.CONTROL_PPM)
                c = g.Circle(center,3)
                c.setFill('orange')
                c.draw(self.win)
                print('[I] Centroid:'+str(path_root))
        else:
            #use current pos for paths
            path_root = self.get_latest_gps()

        #current pos of agent for time calcs.
        current_pos = self.get_latest_gps()
        #the value of a point is all the values on the path to that point
        path_stds = []
        target_stds = []
        start = time.time()
        #actual paths as sepatate lists
        paths = []
        #put all paths into a single list for GP to regress. Looping over paths
        #takes forever and is inefficient
        flat_paths = []
        path_lens = []
        #known targets of other agents
        other_targets = [other_agent['target'] for other_agent in self.other_agents.values()]

        #TODO current_pos'u da ekle hesaplara

        #consider the candidates
        for candidate in targets:
            dist = gm.euclid_distance(path_root, candidate)
            path = gm.subdividePath([path_root, candidate], int(dist))
            #we dont want paths that will go near a known target of another agent
            too_close = False
            for point in path:
                for other_target, other_target_value in other_targets:
                    if other_target is None or other_target_value == -1:
                        continue
                    dist = gm.euclid_distance(point, other_target)
                    if dist < config.TARGET_PROXIMITY_LIMIT:
                        too_close = True
                        break
                if too_close:
                    break

            if not too_close:
                flat_paths.extend(path)
                paths.append(path)
                path_lens.append(len(path))

        if len(flat_paths) > 0:
            #regress for all paths at once
            all_means, all_stds = self.gp.regress(flat_paths)
        else:
            #no possible paths found
            return None,-1


        #separate the paths again
        for i in range(len(path_lens)):
            if i == 0:
                prev = 0
                stds = all_stds[0:path_lens[i]]
                means = all_means[0:path_lens[i]]
            else:
                prev = path_lens[i-1]
                stds = all_stds[prev + 1 : prev + path_lens[i]]
                means = all_means[prev + 1 : prev + path_lens[i]]

            if config.AVOID_LAND:
                #dont consider a path that goes through a land
                #TODO instead, pathfind around said land
                scaled_means = self.scale_means(means)
                contains_land = map(lambda sm: sm < config.DEPTH_LIMIT, scaled_means)
                if any(contains_land):
                    pass

            path_stds.append(sum(stds))
            target_stds.append(stds[-1]) #last one is the candidate always
        print('[I] took '+str(time.time()-start)+' seconds for path stds')

        #once the paths are filtered, find out the remaining targets
        targets = [pathh[-1] for pathh in paths]


        if config.CARE_ABOUT_TTR:
            #time to reach these candidate points
            times = self.time_to_reach(current_pos, targets)
            times = np.array(times)
            path_stds = np.array(path_stds)
            target_stds = np.array(target_stds)
            #value of the candidates. quickest most deviating point please
            values = path_stds / times
        else:
            values = path_stds

        best_point = np.argmax(values)
        print('[I] chosen target value',values[best_point])
        return targets[best_point], values[best_point]


    def try_setting_target(self):
        """
        tries to find and choose a target once. If fails, increases radius and returns
        without re-trying with the new radius. If succeeds, resets the radii and
        sets the target.
        """
        #dont move while calculating a target
        self.send_to_body('set_stop')

        if config.USE_PURE_TVIC:
            #only use pure tahirovic w/o the GP parts

            #set enough points to a really large value. tvic is fast enough
            #to use all available measurements all the time.
            self.enough_points = 9999999999999999
            mments = self.get_enough_measurements()
            for m in mments:
                x,y = m[:2] #tvic does not care about depth
                #fill the tvic matrix
                self.fill_tvic([x,y])

            #tvic matrix is now filled, find the mean of unexplored space

            #indicies of elements where the value is 0 = unexplored
            ix = np.in1d(self.tvic_matrix.ravel(), [0]).reshape(self.tvic_matrix.shape)
            #list of xs and ys in columns
            unexp_idx = np.where(ix)
            #mean of unexplored area
            mean = np.mean(unexp_idx, axis=1)
            #mean is in tvic space, get that into real
            mean = self.to_real(mean)
            print('[I] Mean:'+str(mean))

            if config.T_DRAW_CENTROID and not config.HEADLESS:
                xx,yy = mean
                c = g.Circle(g.Point(xx*config.PPM, yy*config.PPM), 0.5*config.PPM)
                c.setFill('orange')
                c.draw(self.win)

            current_pos = self.get_latest_gps()
            #some candidate points around the agent
            candidates = u.make_circle_targets(center=current_pos,
                                               radii = range(self.start_range ,self.end_range),
                                               count = self.circle_count)
            #filter candidates that are inside the polygon bounds
            candidates = filter(lambda p: gm.ptInPoly(self.bounds, p), candidates)
            candidates = np.array(candidates)
            #collect unexplored targets and their distances to mean
            targets = []
            values = []
            tvic_candidates = self.to_tvic(candidates)
            for p in tvic_candidates:
                #get the tvic version of p to check for exploredness
                xx,yy = int(p[0]),int(p[1])
                if xx >= self.tvic_matrix.shape[0]:
                    xx = self.tvic_matrix.shape[0]-1
                if yy >= self.tvic_matrix.shape[1]:
                    yy = self.tvic_matrix.shape[1]-1
                if self.tvic_matrix[xx,yy] == 0:
                    rp = self.to_real([p])[0]
                    m_dist = gm.euclid_distance(rp, mean)
                    s_dist = gm.euclid_distance(rp, current_pos)
                    targets.append(rp)
                    values.append(s_dist**2 + (1./ (m_dist**2)))

            if targets is not None and len(targets) > 0:
                #choose the point furthest away from mean
                chosen_idx = np.argmin(values)
                #this target is in tvic matrix space, scale back to real space
                target = targets[chosen_idx]
                self.target = target
                self.start_range = config.DEFAULT_START_RANGE
                self.end_range = config.DEFAULT_END_RANGE
                self.circle_count = config.DEFAULT_CIRCLE_COUNT
                return True
            else:
                #no targets found, increase search radius
                print('[I] Increasing search radius from '+str(self.end_range)+
                    ' to '+str(self.end_range + config.SEARCH_INCREMENT))
                self.start_range = self.end_range
                self.end_range += config.SEARCH_INCREMENT
                self.circle_count *= 2
                if self.end_range >= self.max_range:
                    print('[I] Maximum radius achieved, no points found.')
                    print('[M] Going remote, we are done')
                    self.mode = 'remote'
                    return False
            return False


        #generate targets around the agent
        targets = self.generate_targets()
        #do we have enough targets to make a decision?
        if targets is not None and len(targets) >= config.MIN_UNEXPLORED:
            #we have enough targets, choose one and set it
            self.target, self.target_value = self.choose_target(targets)
            if self.target is None:
                return False
            #reset the radii in case they were increased for this particular search
            self.start_range = config.DEFAULT_START_RANGE
            self.end_range = config.DEFAULT_END_RANGE
            self.circle_count = config.DEFAULT_CIRCLE_COUNT
            return True
        else:
            #we dont have enough targets, increase search radius and pass
            print('[I] Increasing search radius from '+str(self.end_range)+
                    ' to '+str(self.end_range + config.SEARCH_INCREMENT))
            self.start_range = self.end_range
            self.end_range += config.SEARCH_INCREMENT
            self.circle_count *= 2
            if self.end_range >= self.max_range:
                print('[I] Maximum radius achieved, no points found.')
                #number of points we can use at most. This is the number of mments
                #this agent knows, including the ones from others.
                max_points = len(self.measurements)
                for other_agent in self.other_agents.itervalues():
                    max_points += len(other_agent['mments'])

                #if we already used ALL of the measurements, and reached max radius,
                #we are totally done.
                if self.enough_points >= max_points:
                    print('[I] Used all measurements too, no target found')
                    print('[M] Going remote')
                    self.mode = 'remote'
                    self.start_range = config.DEFAULT_START_RANGE
                    self.end_range = config.DEFAULT_END_RANGE
                    self.circle_count = config.DEFAULT_CIRCLE_COUNT
                    self.enough_points = config.DEFAULT_ENOUGH
                    return False

                print('### self enough',self.enough_points, 'max pts', max_points)
                #we did not use ALL of the measurements yet, try that
                #simply double the number of points to use
                #this number of points will be used until a target is found
                #number of points will be reset when a target is found
                self.enough_points *= 1.5
                #reset the radii in case they were increased for this particular search
                self.start_range = config.DEFAULT_START_RANGE
                self.end_range = config.DEFAULT_END_RANGE
                self.circle_count = config.DEFAULT_CIRCLE_COUNT
            return False


    def broadcast_latest_measurement(self):
        """
        broadcasts the latest measurement with known depth, position and velocity
        also adds bcast the current target so others can avoid selecting the same target
        """
        #do we even have a measurement yet?
        if len(self.measurements) < 1:
            return None, None

        #try not to flood the udp buffers
        dt = time.time() - self.last_mment_broadcast_time
        if dt < config.MMENT_INTERVAL:
            return

        #search for the latest complete measurement
        gps = None
        sonar = None
        m = None
        id = None
        idx = 0
        while gps is None or sonar is None:
            idx += 1
            #got that many measurements?
            if len(self.measurements) <= idx:
                return
            m = self.measurements[-idx]
            id = self.m_ids[-idx]
            gps = m[:2]
            sonar = m[2]

        pos = (m[0],m[1],self.V[0],self.V[1],m[2])
        self.broadcast({'from':self.id,
                        'bcast_type':'mment',
                        'pos':pos,
                        'id':id,
                        'target':[self.target,self.target_value]})

        self.last_mment_broadcast_time = time.time()


    def missing_mments_other(self, other_id):
        """
        returns a set of ids that are missing from the sequence of ids we have
        assuming the largest id is indeed the last one.

        Returned set is unordered, but that is irrelevant.
        """
        #the measurements we have from this agent
        other_agent = self.other_agents.get(other_id,None)
        #do we have ANY at all?
        if other_agent is None:
            #we have no mments from this agent at all
            return None

        #just the measurements please
        mments = other_agent.get('mments')

        if len(mments) < 1:
            #we somehow have some idea what this agent is, but have no mments from it
            return None

        #we have some measurements
        #the measurements have increasing sequence numbers, 'ids'
        #convert to ints
        ids = map(int, mments.keys())
        #sets make diffing easy.
        ids = set(ids)
        #last known mment id
        last = max(ids)
        #given the last measurement id, we -should- have the entire range of measurements
        #except 0th one. That is borked for some reason, cba to find out why
        #not critical at all.
        ideal_ids = set(range(1,last))
        #missing ids are the difference between ideal and existing
        missing = ideal_ids.difference(ids)

        return missing,last


    def broadcast_missing(self):
        """
        broadcast the missing ids for each agent. Any other agent that can read
        this will respond with the missing values.

        The situation where we know of an agents id but have NO mments from it is
        not handled. Assuming we 'know' of agents first when we hear a measurement from
        them. Which should be very frequent when they are in range.

        if after broadcasting, a listening agent explodes with a pickle error,
        do what you did for filling here.
        """
        #should we bcast now?
        dt = time.time() - self.last_missing_broadcast_time
        if dt < config.MISSING_INTERVAL:
            #nope
            return
        #yup
        for other_agent in self.other_agents.values():
            missing,last = self.missing_mments_other(other_agent['id'])
            if missing is not None and len(missing) > 0:
                #we are missing some stuff, broadcast this fact
                #set udp receive buffer to max (64k) and send large but few packets
                #instead. use the fill type for that. do not send the whole thing
                #as a single datagram.
                self.broadcast({'from':self.id,
                                'bcast_type':'missing',
                                'missing_from':other_agent['id'],
                                'missing':missing,
                                'last':last})

                print('[I] Bcasted missing values for '+\
                        other_agent['id']+':'+\
                        str(len(missing)))

        #record the time of bcast
        self.last_missing_broadcast_time = time.time()


    def broadcast_filling(self, missing_from, missing, last):
        """
        broadcasts the filling values with whatever data this agent has.
        missing_from is the agent id that this data is from
        missin is a set of measurement ids that is requested
        last is the last known id by the broadcasting agent

        the broadcast is targetless, recipient is anyone that listens.
        """
        fill = []
        #is the other guy missing data from this agent?
        if missing_from == self.id:
            #missing data from this, send self data
            for mment_id in missing:
                #self measurements are id-indexed
                mment = self.measurements[mment_id]
                fill.append([mment_id,mment])

        else:
            #missing data from an other agent that we might know about
            other_agent = self.other_agents.get(missing_from)
            #do we know this agent?
            if other_agent is not None:
                #we do
                mments = other_agent['mments']
                for mment_id in missing:
                    mment = mments.get(mment_id)
                    #did we have this particular measurement?
                    if mment is not None:
                        #velocities of other agents ARE stored, no need for filler
                        fill.append([mment_id,mment])
                    #if we did not, nothing to do

        #broadcast this filling data, hope the other guy(s) read it
        #set udp receive buffer to max (64k) and send large but few packets
        #instead. use the fill type for that.
        partial = []
        for id,mment in fill:
            if len(partial) < 10:
                #truncate the coords of the mment, no need to send picometers when
                #gps has meters worth of inaccuracies.
                #also avoid float problems by converting to strings instead
                x = str(mment[0])[:4]
                y = str(mment[1])[:4]
                d = str(mment[2])[:6]
                strmment = [x,y,d]
                partial.append([id,strmment])
            else:
                self.broadcast({'from':self.id,
                                'fill_to':missing_from,
                                'bcast_type':'fill',
                                'fill':partial})
                partial = []
        #bcast whatever is left in partial
        if len(partial) > 0:
            self.broadcast({'from':self.id,
                            'fill_to':missing_from,
                            'bcast_type':'fill',
                            'fill':partial})

        print('[I] Bcasted filling values for: '+missing_from+':'+str(len(fill)))


    def will_collide(self, other_pos, other_V):
        """
        returns true if this agents velocity is inside the other agents VO.
        """
        #TODO fix dis
        self_pos = np.array(self.get_latest_gps())
        self_V = self.V
        other_pos = np.array(other_pos)
        other_V = np.array(other_V)
        #vector that points at the other agent center from this center
        center_vector = other_pos - self_pos
        #distance between centers
        dist = gm.euclid_distance(self_pos, other_pos)
        #angle between center vector and the tangent line to other agent
        theta = np.arcsin((self.vo_r + self.other_r)/dist)
        #angle of the center vector
        alpha = np.arcsin(center_vector[1]/dist)

        #angle of the tangent lines
        t1_a = alpha+theta
        t2_a = alpha-theta

        #make a really large triangle out of these two tangent lines to make
        #checking inclusion easier.

        #one side
        t1 = np.array([np.cos(t1_a), np.sin(t1_a)]) * 10000
        t1 += self_pos
        #other side
        t2 = np.array([np.cos(t2_a), np.sin(t2_a)]) * 10000
        t2 += self_pos
        #three vertices
        VO = np.array([self_pos, t1, t2])
        VO += other_V

        print(t1_a,t1,t2_a,t2,VO)


    def fill_tvic(self, point):
#        if config.PAINT_TAHIROVIC and not config.HEADLESS:
#            self.bg.setPixel(int(point[0]*config.PPM)+config.CONTROL_WIN_SIZE/2,
#                             int(point[1]*config.PPM)+config.CONTROL_WIN_SIZE/2,
#                             'gray')


        tvic_center = self.to_tvic(point)
        tvic_center = np.round(tvic_center)
        r = config.TVIC_RADIUS
        for x in range(int(tvic_center[0])-r, int(tvic_center[0])+r):
            for y in range(int(tvic_center[1])-r, int(tvic_center[1])+r):
                xx = x
                yy = y
                if xx >= self.tvic_matrix.shape[0]:
                    xx = self.tvic_matrix.shape[0]-1
                if yy >= self.tvic_matrix.shape[1]:
                    yy = self.tvic_matrix.shape[1]-1
                if xx < 0:
                    xx = 0
                if yy < 0:
                    yy = 0

                if gm.euclid_distance(point, self.to_real([xx,yy])) <= r:
                    self.tvic_matrix[xx,yy] = 1


###############################################################################
# actual control that is run continuously
###############################################################################
    def control(self):
        #time since starting control
        self.time = time.time() - self.start_time

        #draw the agent on display
        self.draw_self()
        #draw the other agents on display
        self.draw_others()

        #######################################################################
        # sensor readings, these are not in functions since this method is ran
        # at high frequency. Python is horrible with function calls.
        #######################################################################
        #from the incoming queue, only read the ones addressed to this agent
        messages = u.msgs_to_self(self.addr, self.consumer)
        #if there are messages to this agent, handle'em
        if len(messages) > 0:
            for message in messages:
                sensor_type = message.get('type')
                sensor_value = message.get('value')
                if sensor_value is not None and sensor_type is not None:
                    if sensor_type == 'network':
                        #stuff read from the network should be data from other agents
                        #string id of agent sending the broadcast
                        other_id = sensor_value.get('from')

                        #broadcasts have types 'missing' and 'mment' or 'fill'
                        bcast_type = sensor_value.get('bcast_type')

#handle filling broadcasts.
#these are bcasts that will fill some gap in measurements of some agent
#it might not be FOR this agent, but this agent might fill a few holes
#here and there maybe.
                        if bcast_type == 'fill':
                            #ignore self-broadcasts
                            if other_id is not None and other_id != self.id:
                                fill_to = sensor_value.get('fill_to')
                                strfill = sensor_value.get('fill')

                                fill = []
                                #convert the str'ified values to back to floats
                                for id,mment in strfill:
                                    if mment[0] == 'None':
                                        #gps can be None, new sonar arrived before gps
                                        x = None
                                        y = None
                                    else:
                                        x = float(mment[0])
                                        y = float(mment[1])

                                    if mment[2]=='None':
                                        #depth can be None, new gps arrived before sonar
                                        d = None
                                    else:
                                        d = float(mment[2])
                                    fill.append([id,[x,y,0,0,d]])

                                #which agent does this data belong to?
                                other_agent = self.other_agents.get(fill_to)
                                if other_agent is None:
                                    #its an agent we havent met before!
                                    new_agent = {'id':fill_to, 'mments':{}}
                                    #record this new agent
                                    self.other_agents[other_id] = new_agent

                                #we know this guy. for sure.
                                other_agent = self.other_agents.get(fill_to)
                                for m_id,m in fill:
                                    #fill in the measurement map of this agent
                                    other_agent['mments'][m_id] = m


                        #handle missing value broadcasts
                        if bcast_type == 'missing':
                            #ignore self-broadcasts
                            if other_id is not None and other_id != self.id:
                                missing_from = sensor_value.get('missing_from')
                                missing = sensor_value.get('missing')
                                last = sensor_value.get('last')
                                #broadcast whatever we have of this missing data
                                self.broadcast_filling(missing_from, missing, last)

                        #handle measurement broadcasts
                        if bcast_type == 'mment':
                            #measurement, x,y,vx,vy,depth
                            bcast_mment = sensor_value.get('pos')
                            #measurement id, a simple sequence number for later diffing
                            bcast_m_id = sensor_value.get('id')
                            #current target and value of the agent
                            tgt = sensor_value.get('target')

                            #care about bcasts only from others
                            if other_id is not None and other_id != self.id:
                                #simulating a network range
                                self_pos = self.get_latest_gps()
                                if self_pos is not None:
                                    dist = gm.euclid_distance(self_pos, bcast_mment[:2])
                                else:
                                    #no clue where we are, assume no-connection
                                    dist = config.NETWORK_RANGE+1

                                #dist can be None, early stopping of booleans ftw
                                if not config.SIMULATE_NETWORK_BREAKAGE or dist <= config.NETWORK_RANGE:
                                    #do we have a record for this agent?
                                    other_agent = self.other_agents.get(other_id)
                                    if other_agent is None:
                                        #we dont have a record for this agent, make a new one
                                        #all we need is an id and the measurements of that agent
                                        new_agent = {'id':other_id,
                                                     'mments':{bcast_m_id:bcast_mment},
                                                     'target':tgt}
                                        #record this new agent
                                        self.other_agents[other_id] = new_agent
                                    else:
                                        #we already know about this agent, update its measurements
                                        other_agent['mments'][bcast_m_id] = bcast_mment
                                        other_agent['target'] = tgt


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


        #broadcast the last measurement so the other agents can avoid us, record us w/e
        self.broadcast_latest_measurement()
        #broadcast the missing values of other agents
        self.broadcast_missing()

        ######################################################################
        # key controls
        ######################################################################
        if not config.HEADLESS:
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
            if key == 'b':
                for other_agent in self.other_agents.values():
                    mments = other_agent['mments']
                    for mment in mments.values():
                        x,y = mment[0],mment[1]
                        x*=config.CONTROL_PPM
                        y*=config.CONTROL_PPM
                        c = g.Point(x,y)
                        c.setFill('red')
                        c.setOutline('red')
                        c.draw(self.win)
            if key == 'g':
                if config.USE_PURE_TVIC:
                    plt.matshow(self.tvic_matrix)
                    plt.show(block=False)
                else:
                    fig, means, stds = self.gp.show_surface(show=False)
                    plt.matshow(stds)
                    plt.colorbar()
                    plt.show(block=False)

            if key == 'S':
                self.tcp.send('$MSSTA,*00', data_type='command')

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
                    #this is expected from the operator when running on a
                    #physical body.
                    self.send_to_body('set_max_speed')


            if self.should_reset_target():
                self.target = None
                #a new target will be set in the next cycle (very soon)


        if not config.HEADLESS:
            g.update(config.UPDATE_FPS)

        #we are not done yet.
        return False


    def should_reset_target(self):
        """
        return true if this agent should re-set its target
        """
        #check if our target is too close to another's.
        #decide who keeps their target by checking IDs.

        if self.target is not None:
            for other_agent in self.other_agents.itervalues():
                other_target = other_agent['target'][0] #target,value
                if other_target is not None:
                    dist = gm.euclid_distance(self.target, other_target)
                    if dist < config.TARGET_PROXIMITY_LIMIT:
                        if self.id < other_agent['id']:
                            print('[T] Target too close, I have the lower ID, should re-set target')
                            return True





        return False

    def set_body_target(self, target):
        x,y = target
        self.send_to_body('set_target,'+str(x)+','+str(y))
        print('[T] set target to'+ str(self.target))


    def send_to_body(self,msg):
        """
        convenience method to send requests to the body
        should be used to control the body of the agent
        """
        if config.SIMULATION:
            self.producer.send(u.msg(self.id+'-body',msg))
        else:
            if msg == 'set_max_speed':
                #dont do this. if this is not a simulation the operator
                #is expected to use remote control to gather the initial
                #measurements manually.
                pass

            #the physical body has no 'stop' function, so... set target
            #to current position instead and execute that.
            if msg == 'set_stop':
                msg = '$MSSTO,*00\r\n'
                self.tcp.send(msg,data_type = 'command')


            if msg.find('set_target') != -1:
                #set target has some stuff to parse into NMEA, tcp handles that
                parts = msg.split(',')
                msg = parts[0]
                x = float(parts[1])
                y = float(parts[2])
                latlon = config.XY_TO_LATLON([x,y])
                #need to get values out of hte numpy arrays.
                msg += ','+str(latlon[0][0])+','+str(latlon[1][0])
                self.tcp.send(msg,data_type = 'command')

    def broadcast(self, msg):
        """
        sends a message for anyone in range to listen to

        has nothing to do with the body of the agent so running in sim or
        physical body is the same. No need for tcp comms with the body here.
        """
        self.producer.send(u.msg('broadcast', msg))


    def save_trace(self):
        """
        save the gps and sonar measurements to a file for reasons
        """
        filename=config.TRACE_DIR+\
                        '_'+\
                        self.id+\
                        '_trace_'+\
                        config.SUFFIX

        noneless = filter(lambda m: m[0] is not None and\
                                    m[1] is not None and\
                                    m[2] is not None,
                            self.measurements)

        a = np.array(noneless)
        np.savetxt(filename,a)
        print('[I] trace saved to; '+filename)

    def save_other_traces(self):
        #TODO fix dis
        """
        save the measurements from others
        """
        filename = config.TRACE_DIR+self.id+'_trace_others_'+config.SUFFIX
        with open(filename,'w') as f:
            for agent in self.other_agents.values():
                ms = agent.get('mments')
                if ms is None or len(ms) < 1:
                    continue

                for x,y,d in ms.values():
                    f.write(str(x)+','+str(y)+','+str(d)+'\n')
        print('[I] others measurements saved to; '+filename)


if __name__=='__main__':
    try:
        id = sys.argv[1]
        ip = sys.argv[2]
        port = sys.argv[3]
    except:
        id = '0'
        ip = '10.13.20.23'
        port = '12'
    agent = Agent(id,body_ip = ip,body_port = port)
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
        agent.save_trace()
    else:
        print('[I] Stopped')
        agent.win.close()
        for proc in agent.sensor_processes:
            proc.kill()
        agent.save_trace()

    print('[I] Stopped')
    agent.win.close()
    for proc in agent.sensor_processes:
        proc.kill()
    agent.save_trace()
    agent.send_to_body('set_stop')

    if not config.HEADLESS:
        filename=config.TRACE_DIR+\
                        '_'+\
                        str(id)+\
                        '_trace_'+\
                        config.SUFFIX
        trace = np.loadtxt(filename)
        plt.plot(trace[:,0],trace[:,1])
        plt.show(block=False)



