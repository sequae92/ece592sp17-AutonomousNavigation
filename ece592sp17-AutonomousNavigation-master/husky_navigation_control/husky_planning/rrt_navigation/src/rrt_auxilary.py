#!/usr/bin/env python
#import cv2
import time
import numpy as np
from random import randint
from math import sqrt

class Tree(object):

    def __init__(self, init_pt, tgt_pt ,step, hit_tgt, height, width, map_array, threshold):
        #save target point
        self.tgt = tgt_pt
        #save starting point
        self.ini = init_pt
        #initial RRT node
        self.nodes = [init_pt]
        #initial RRT line(connect 2 node)
        self.lines = []
        #set RRT step length
        self.step = step
        #check RRT reaches target
        self.hit_tgt = hit_tgt
        #save map height and width and map array
        self.height = height
        self.width = width
        self.map = map_array
        #this threshold determind the cell is empty or not
        self.threshold = threshold
        #check the target point is at feasible region
        self.target_feasible = check_feasible(self.tgt, self.threshold,self.map)
        #if target is not at feasible region, it need temperay target for navigatioon
        if self.target_feasible == False:
            self.temp_target =None

    #this for generating next node for RRT
    def get_next_node(self,close_pt):
        #get the random point
        p1 = self.rand_pt
        #find the distance between random point and the closest node of RRT to random point
        dist = distance(close_pt, p1)
        #compute the ratio of one step to the distance
        ratio = self.step/(dist)

        ratio_x = ratio*(self.rand_pt[0]-close_pt[0])
        ratio_y = ratio*(self.rand_pt[1]-close_pt[1])
        #print ratio_x, ratio_y
        # this is for dealing close to boundary
        for i in range(1, 101):
            #next_node_x,next_node_y = 0,0
            next_node_x = close_pt[0]+ ratio_x*(i/100.0)
            next_node_y = close_pt[1]+ ratio_y*(i/100.0)
            #print (next_node_x,next_node_y)
            next_node_x = int(round(next_node_x))
            next_node_y = int(round(next_node_y))

            temp_pt =  (next_node_x,next_node_y)
            #print (next_node_x,next_node_y)
            if check_feasible(temp_pt, self.threshold, self.map) == False:
                #self.nodes.append(next_node)
                break
                #return next_node
        next_node = (next_node_x, next_node_y)
        if next_node != close_pt:
            self.nodes.append(next_node)
            parent, child = close_pt, next_node
            self.lines.append((parent, child))
        return next_node

    #this check the point is whith certain radius feasible region
    def check_circle_feasible(self, pt):
        center_x = pt[0]
        center_y = pt[1]
        center = (center_x, center_y)
        #set radius
        radius = int(self.step/1.5)
        #use stochastic method to check, sample 1/4 area
        for i in range(3*radius*radius):
            rand_pt_x = randint(center_x-radius, center_x+radius)
            rand_pt_y = randint(center_y-radius, center_y+radius)
            rand_pt = (rand_pt_x, rand_pt_y)
            dist_center = distance(center, rand_pt)
            while dist_center > radius:
                rand_pt_x = randint(center_x-radius, center_x+radius)
                rand_pt_y = randint(center_y-radius, center_y+radius)
                rand_pt = (rand_pt_x, rand_pt_y)
                dist_center = distance(center, rand_pt)
            if check_feasible(rand_pt, self.threshold, self.map) == False:
                return False
        return True

    def generate_rand(self,boundary_x, boundary_y):
        flag = True
        while flag:
            rand_x = randint(0,(boundary_x-1))
            rand_y = randint(0,(boundary_y-1))
            rand_point = (rand_x,rand_y)
            dist_pass = True
            for node in self.nodes:
                dist = distance(node, rand_point)
                if dist <= self.step:
                   dist_pass = False
            if dist_pass == True:
                flag = False
        self.rand_pt = (rand_x, rand_y)
        return self.rand_pt

    def close_node(self):
        close_pt = None
        close_dist = max(self.height, self.width)*1.5

        for node in self.nodes:
            dist = distance(node, self.rand_pt)

            if dist < close_dist:
                close_pt = node
                close_dist = dist
        return close_pt
    def near_target(self):
        margin = self.step
        self.target_feasible
        #print self.tgt
        if self.target_feasible:
            for node in self.nodes:

                dist = distance(node, self.tgt)

                if dist <= margin :
                    self.hit_tgt = True
                    close_tgt = node
                    return self.hit_tgt, close_tgt
        self.hit_tgt = False
        close_tgt = None
        return self.hit_tgt, close_tgt

def distance(p0, p1):
    return sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def rand_pt(boundary_x, boundary_y):
    rand_x = randint((boundary_x-1)/2-500,(boundary_x-1)/2+500)
    rand_y = randint((boundary_y-1)/2-500,(boundary_y-1)/2+500)
    return (rand_x, rand_y)

def check_feasible(pt, threshold,array):
    ind_row = pt[1]
    ind_col = pt[0]
    #print px
    pt_value = array[ind_row][ind_col]
    if pt_value < threshold and pt_value >= 0:
        return True
    elif pt_value < 0:
        return False
    else:
        return False

def target(boundary_x, boundary_y):
    tgt = rand_pt(boundary_x, boundary_y)
    while True:
        if check_feasible(tgt):
            break
        else:
            tgt = rand_pt(boundary_x, boundary_y)
    return tgt

def connect_target(object):
    min_dist = 4000
    close_node = None
    for node in object.nodes:

         flag = check_line(node, object.tgt, object.threshold, object.map)
         dist  = distance(node, object.tgt)
         if dist < min_dist and flag == True:
             min_dist = dist
             close_node = node

    if close_node != None:
        return close_node
    else:
        for node in object.nodes:
             dist  = distance(node, object.tgt)
             if dist < min_dist:
                 min_dist = dist
                 close_node = node
        return close_node

def check_line(p0, p1, threshold, array):
    dist  = int(distance(p0, p1))
    ratio_x = p1[0] - p0[0]
    ratio_y = p1[1] - p0[1]
    for i in range(1, dist+1):
        way_point_x = int(p0[0] +ratio_x*(i/float(dist)))
        way_point_y = int(p0[1] +ratio_y*(i/float(dist)))
        way_point = (way_point_x, way_point_y)
        if check_feasible(way_point, threshold, array)==False:
            return False
    return True

def find_path(near_tgt_node, tgt_pt,line_list):

    if len(line_list)==1:
        path = line_list
    else:
        root = line_list[0][0]

        end = near_tgt_node
        path = [(near_tgt_node, tgt_pt)]
        parent = None
        child = end
        while parent != root:
            parent_temp = None
            for line in line_list:
                line_parent = line[0]
                line_child = line[1]
                if line_child == child:
                    parent = line_parent
                    path.append((line_parent, line_child))
            child = parent
        path.reverse()
    return path
