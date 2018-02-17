#!/usr/bin/env python

import rospy
import math
import numpy as np
from random import randint
from math import sqrt
from rrt_navigation.msg import waypoints
from rrt_auxilary import *
from geometry_msgs.msg import Twist,TransformStamped, PoseStamped
import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String

"""
def back_to_1d_array(array):
    height = len(array)
    width = len(array[0])
    temp = np.reshape(array,(height*width))
    temp.astype(int)

    return temp.tolist()
"""

def reshape_array(width, height, array):

    return np.reshape(array,(height, width))
"""
def map_obstacle_expand(array, resolution, threshold):
    #create boolean array for obstacle, it obstacle, true
    #bool_array = (array>threshold)
    #index of cell of obstacle
    r,c = np.where(array[:]>threshold)
    #create pair r,c
    pair = []
    for num in range(len(r)):
        pair.append((r[num], c[num]))
    temp_array = array
    num_row = len(array)
    num_col = len(array[0])
    #print num_row, num_col
    #keep 1.5 meter distance
    expand_cell = int(1.5/resolution)
    #print expand_cell
    for cell in pair:
        #print cell
        slice_r_center = cell[0]
        slice_c_center = cell[1]
        slice_r_start = slice_r_center-expand_cell
        slice_r_end   = slice_r_center+expand_cell+1
        slice_c_start = slice_c_center-expand_cell
        slice_c_end   = slice_c_center+expand_cell+1
        slice_r = range(slice_r_start,slice_r_end,1)
        slice_r_list = [x for x in slice_r if x>=0 and x<num_row]
        slice_c = range(slice_c_start,slice_c_end,1)
        slice_c_list = [x for x in slice_c if x>=0 and x<num_col]
        #print slice_c_list
        for i in slice_r_list:
            try:
                temp_array[i][slice_c_list[0]:slice_c_list[-1]] = 100
            except:
                print cell, slice_c_list

            #for j in slice_c_list:

            #    print i,j
            #    temp_array[i][j]=1
    #return temp_array
   """

def meter_to_cell(origin_meter, pos_meter, resolution):
    x_cell = int(round((pos_meter[0]-origin_meter[0])/resolution))
    y_cell = int(round((pos_meter[1]-origin_meter[1])/resolution))
    return (x_cell, y_cell)

def cell_to_meter(origin_meter, cell_pose, resolution):
    x_meter = cell_pose[0]*resolution+origin_meter[0]
    y_meter = cell_pose[1]*resolution+origin_meter[1]
    return (x_meter, y_meter)

"""
still need consider the goal in unknow area or infeasible zone

"""
class RRTList():
    def __init__(self):
        rospy.init_node('RRT_info',  anonymous=True)

        self.rate = 5
        #self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=1)
        self.pub = rospy.Publisher('/husky_controller/velocity_control', Twist,queue_size=1)
        self.initialize()
        #target (x,y)
        goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        self.target_point_x_meter = goal_msg.pose.position.x
        self.target_point_y_meter = goal_msg.pose.position.y
        self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)


        #get current robot position(get msg for one time)
        pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
        self.starting_x_meter = pose_msg.transform.translation.x
        self.starting_y_meter = pose_msg.transform.translation.y
        self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
        #get map info
        map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
        self.map_width = map_info_msg.info.width
        self.map_height = map_info_msg.info.height
        self.map_resolution = map_info_msg.info.resolution
        self.map_array = reshape_array(self.map_width, self.map_height, map_info_msg.data)
        self.map_origin_meter = (map_info_msg.info.origin.position.x, map_info_msg.info.origin.position.y)
        #convert current robot pose from meter to cell
        self.starting_robot_cell = meter_to_cell(self.map_origin_meter, self.starting_robot_meter, self.map_resolution)
        #convert target point from meter to cell
        self.target_point_cell = meter_to_cell(self.map_origin_meter, self.target_point_meter, self.map_resolution)

        self.step = 30
        self.hit_tgt = False
        self.threshold = 25
        self.points_lst_meter = []
        self.rrt = Tree(self.starting_robot_cell,
            self.target_point_cell, self.step, self.hit_tgt,
            self.map_height, self.map_width, self.map_array, self.threshold)
        self.pub_rrt = rospy.Publisher('/way_point_info', waypoints,queue_size=1, latch=True)

        """
        #test obstacle expansion

        self.obstacle_map = rospy.Publisher('/map_obstacle_expand', OccupancyGrid,queue_size=5)
        map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
        new_map = OccupancyGrid()
        new_map.info = map_info_msg.info
        data = map_obstacle_expand(self.map_array, self.map_resolution, self.threshold)
        new_map.data = back_to_1d_array(data)
        self.obstacle_map.publish(new_map)
        #print "e"
        ###

    def update_obstacle_map(self):
        map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
        new_map = OccupancyGrid()
        new_map.info = map_info_msg.info
        data = reshape_array(self.map_width, self.map_height, map_info_msg.data)
        data = map_obstacle_expand(data, self.map_resolution, self.threshold)
        new_map.data = back_to_1d_array(data)
        self.obstacle_map.publish(new_map)
    """
    def initialize(self):
        string = Twist()
        rate = rospy.Rate(self.rate)
        print "start initialization"
        #backward
        for i in range(15):
            string.linear.x = -0.3
            string.angular.z = 0
            self.pub.publish(string)
            rate.sleep()
        #forward
        for i in range(15):
            string.linear.x = 0.3
            string.angular.z = 0
            self.pub.publish(string)
            rate.sleep()
        #rotate
        for i in range(15*8):
            string.linear.x = 0
            string.angular.z = 0.36
            self.pub.publish(string)
            rate.sleep()
        print "initialization completed"
        print "waiting for goal from move_base"

    def rrt_list_update(self):

        # if list is empty, run RRT
        if self.points_lst_meter == []:
            i = 0
            #if RRT not reach target and number of RRT node < 100
            while len(self.rrt.nodes) <= 90 and self.rrt.hit_tgt == False and i <= 100000:
                i = i+1
                #print i
                # generate a random point

                self.rrt.generate_rand(self.map_width, self.map_height)
            #print "rand point",rrt.rand_pt
                # find the closest node to the random point

                c_pt = self.rrt.close_node()
            #print "close node:", c_pt
                # add one step as new node from the closest node

                self.rrt.get_next_node(c_pt)
            #print rrt.nodes
                # check the RRT node is near target

                #print "node num: ", len(self.rrt.nodes)
                temp = self.rrt.near_target()


            # check taget is at feasible region or not
            # if True
            print "1"
            if self.rrt.target_feasible == True:
                # if number <= 120, it means it reaches target
                if len(self.rrt.nodes) <= 90 and i <= 100000:
                    print "1-1"
                    print "sometimes freeze"

                    closest_node = temp[1]
                    print closest_node
                    path = find_path(closest_node, self.rrt.tgt,self.rrt.lines)
                # if number > 120, it means it dosent reache target
                # stll need to be modified following
                else:
                    print "1-2"
                    closest_node = connect_target(self.rrt)

                    self.rrt.lines.append((closest_node, self.rrt.tgt))
                    path = find_path(closest_node, self.rrt.tgt, self.rrt.lines)

            # if False
            else:
                print "1-3"
                try:
                    closest_node = connect_target(self.rrt)
                    self.rrt.temp_target = closest_node

                    path = find_path(closest_node, self.rrt.temp_target , self.rrt.lines)
                except Exception as error:
                    print error

            print "2"
            point_path = []
            for i in range(len(path)):
                point_path.append(path[i][0][0])
                point_path.append(path[i][0][1])
                if i == len(path)-1:
                    point_path.append(path[i][1][0])
                    point_path.append(path[i][1][1])

            print "target cell position: ", self.rrt.tgt
            if self.rrt.target_feasible == True:
                print "target position in meter: ", self.target_point_meter
            else:
                print "the target point is at unknown or unfeasible region"
                print "navifate to temperay target: ", self.rrt.temp_target

            print "starting at ", self.starting_x_meter, self.starting_y_meter, "meter"
            print "show waypoints"
            # point_path is the list of waypoints in cell unit
            print point_path, "unit: cell"
            points_lst = []
            for i in range(len(point_path)/2):
                (x,y)=cell_to_meter(self.map_origin_meter, (point_path[2*i],point_path[2*i+1]), self.map_resolution )
                points_lst.append(x)
                points_lst.append(y)
            # points_lst is the list of waypoints in meter unit
            print points_lst, "unit: meter"
            self.points_lst_meter = points_lst
            waypoints_msg = waypoints()
            waypoints_msg.mid_points_cell= point_path
            waypoints_msg.list_in_meter = points_lst
            self.pub_rrt.publish(waypoints_msg)

        else:
            current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
            robot_x = current_pose_msg.transform.translation.x
            robot_y = current_pose_msg.transform.translation.y
            if self.rrt.target_feasible == True:

                e_x = self.target_point_x_meter - robot_x
                e_y = self.target_point_y_meter - robot_y
                dist = math.sqrt((e_x)**2+(e_y)**2)
                if dist <= 0.15:
                    #reset target (x,y)
                    print "waiting for goal from move_base"
                    goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
                    self.target_point_x_meter = goal_msg.pose.position.x
                    self.target_point_y_meter = goal_msg.pose.position.y
                    self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)

                    #re-get current robot position(get msg for one time)
                    pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                    self.starting_x_meter = pose_msg.transform.translation.x
                    self.starting_y_meter = pose_msg.transform.translation.y
                    self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
                    #re-get map info
                    map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
                    self.map_width = map_info_msg.info.width
                    self.map_height = map_info_msg.info.height
                    self.map_resolution = map_info_msg.info.resolution
                    self.map_array = reshape_array(self.map_width, self.map_height, map_info_msg.data)
                    self.map_origin_meter = (map_info_msg.info.origin.position.x, map_info_msg.info.origin.position.y)
                    #convert current robot pose from meter to cell
                    self.starting_robot_cell = meter_to_cell(self.map_origin_meter, self.starting_robot_meter, self.map_resolution)
                    #convert target point from meter to cell
                    self.target_point_cell = meter_to_cell(self.map_origin_meter, self.target_point_meter, self.map_resolution)

                    self.points_lst_meter = []
                    self.rrt = Tree(self.starting_robot_cell,
                        self.target_point_cell, self.step, self.hit_tgt,
                        self.map_height, self.map_width, self.map_array, self.threshold)
            else:
                rrt_temp_tgt_meter = cell_to_meter(self.map_origin_meter, self.rrt.temp_target, self.map_resolution)

                e_x = rrt_temp_tgt_meter[0] - robot_x
                e_y = rrt_temp_tgt_meter[1] - robot_y
                dist = math.sqrt((e_x)**2+(e_y)**2)
                if dist <= 0.15:
                    #reset target (x,y)
                    print "waiting for map updating and run RRT"
                    """
                    goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
                    self.target_point_x_meter = goal_msg.pose.position.x
                    self.target_point_y_meter = goal_msg.pose.position.y
                    self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)
                    """
                    #re-get current robot position(get msg for one time)
                    pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                    self.starting_x_meter = pose_msg.transform.translation.x
                    self.starting_y_meter = pose_msg.transform.translation.y
                    self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
                    #re-get map info
                    map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
                    self.map_width = map_info_msg.info.width
                    self.map_height = map_info_msg.info.height
                    self.map_resolution = map_info_msg.info.resolution
                    self.map_array = reshape_array(self.map_width, self.map_height, map_info_msg.data)
                    self.map_origin_meter = (map_info_msg.info.origin.position.x, map_info_msg.info.origin.position.y)
                    #convert current robot pose from meter to cell
                    self.starting_robot_cell = meter_to_cell(self.map_origin_meter, self.starting_robot_meter, self.map_resolution)
                    #convert target point from meter to cell
                    self.target_point_cell = meter_to_cell(self.map_origin_meter, self.target_point_meter, self.map_resolution)

                    self.points_lst_meter = []
                    self.rrt = Tree(self.starting_robot_cell,
                        self.target_point_cell, self.step, self.hit_tgt,
                        self.map_height, self.map_width, self.map_array, self.threshold)
                    self.rrt.target_feasible = self.rrt.check_circle_feasible(self.target_point_cell)

    def spin(self):
        rospy.loginfo("Start RRT")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            #self.update_obstacle_map()
            self.rrt_list_update()

            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down RRT")




def main():
    rrt_list = RRTList()
    print "waiting for goal from move_base"
    rrt_list.spin()


if __name__ == '__main__':

    main()
