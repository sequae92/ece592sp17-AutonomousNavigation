#!/usr/bin/env python

import rospy
import math
import numpy as np
from random import randint
from math import sqrt
from rrt_navigation.msg import waypoints
#from husky_modeswitch.msg import
from husky_modeswitch.msg import husky_controlinfo
from rrt_auxilary import *
from geometry_msgs.msg import Twist,TransformStamped, PoseStamped
import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Int32, Float32
from supplement_fcn_path import sending_points, pt_2_Pose
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


def meter_to_cell(origin_meter, pos_meter, resolution):
    x_cell = int(round((pos_meter[0]-origin_meter[0])/resolution))
    y_cell = int(round((pos_meter[1]-origin_meter[1])/resolution))
    return (x_cell, y_cell)

def cell_to_meter(origin_meter, cell_pose, resolution):
    x_meter = cell_pose[0]*resolution+origin_meter[0]
    y_meter = cell_pose[1]*resolution+origin_meter[1]
    return (x_meter, y_meter)


class RRTList():
    def __init__(self):
        rospy.init_node('RRT_info',  anonymous=True)

        self.rate = 5

        topic_list = rospy.get_published_topics()
        self.topics = []
        for topic in topic_list:
            self.topics.append(topic[0])



        # if none, it means simulation
        if '/husky_modeswitch/controlhardware_interface' in self.topics:
            #for real world

            self.pub = rospy.Publisher('/husky_controller/velocitycontrol',Twist ,queue_size=10)
        else:
            #gazebo
            self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=10)



        self.initialize()

        """
        #### need revise ###
        """
        #get current robot position(get msg for one time)
        try:
            self.odom_name = rospy.get_param('odom_topic')
        except:
            self.odom_name = None


        """
        print "choose odom topic, 0:/odom, 1:/camera/odom, 2:/ORB_SLAM/scaled_odom"
        self.selection_p = raw_input()
        print self.selection_p
        """
        try:
            if self.odom_name == None or self.odom_name == "odom":
                pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
            else:
                pose_msg = rospy.wait_for_message(self.odom_name, Odometry)

        except:
            pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
        """
        if self.selection_p == "0":
            pose_msg = rospy.wait_for_message("/odom", Odometry)
        elif self.selection_p == "1":
            pose_msg = rospy.wait_for_message("/camera/odom", Odometry)
        elif self.selection_p == "2":
            pose_msg = rospy.wait_for_message("/ORB_SLAM/scaled_odom", Odometry)
        else:
            #husky simulation
            pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
            #pose_msg = rospy.wait_for_message("/husky_velocity_controller/odom", Odometry)
        """
        try:
            self.starting_x_meter = pose_msg.pose.pose.position.x
            self.starting_y_meter = pose_msg.pose.pose.position.y
            self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
        except:
            self.starting_x_meter = pose_msg.transform.translation.x
            self.starting_y_meter = pose_msg.transform.translation.y
            self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)

        #get map info
        #try:
            #self.map_name = rospy.get_param('map_topic')
        #except:
        self.map_name = '/map_obstacle_expand'
        map_info_msg = rospy.wait_for_message(self.map_name,OccupancyGrid)
        """
        print "choose map topic, 0:/map, 1:/rtabmap/grid_map, 2:/ORB_SLAM/scaled_odom"
        self.selection_m = raw_input()
        print self.selection_m


        if self.selection_m == "0":
            map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
        elif self.selection_m == "1":
            map_info_msg = rospy.wait_for_message("/rtabmap/grid_map",OccupancyGrid)
        elif self.selection_m == "2":

            map_info_msg  = rospy.wait_for_message("/global_map", OccupancyGrid)
        else:
            #husky simulation
            map_info_msg  = rospy.wait_for_message("/map", OccupancyGrid)
        """


        #target (x,y)
        print "set target by rviz"
        goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
        self.target_point_x_meter = goal_msg.pose.position.x
        self.target_point_y_meter = goal_msg.pose.position.y
        self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)

        self.map_width = map_info_msg.info.width
        self.map_height = map_info_msg.info.height
        self.map_resolution = map_info_msg.info.resolution
        self.map_array = reshape_array(self.map_width, self.map_height, map_info_msg.data)
        self.map_origin_meter = (map_info_msg.info.origin.position.x, map_info_msg.info.origin.position.y)
        #convert current robot pose from meter to cell
        self.starting_robot_cell = meter_to_cell(self.map_origin_meter, self.starting_robot_meter, self.map_resolution)
        #convert target point from meter to cell
        self.target_point_cell = meter_to_cell(self.map_origin_meter, self.target_point_meter, self.map_resolution)



        self.step = 40
        self.hit_tgt = False
        self.threshold = 25
        self.points_lst_meter = []
        self.rrt = Tree(self.starting_robot_cell,
            self.target_point_cell, self.step, self.hit_tgt,
            self.map_height, self.map_width, self.map_array, self.threshold)
        self.pub_rrt = rospy.Publisher('/way_point_info', waypoints,queue_size=10)



    def initialize(self):
        """
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
            string.linear.x= 0.3
            string.angular.z = 0
            self.pub.publish(string)
            rate.sleep()
        #rotate
        for i in range(15*12):
            string.linear.x = 0
            string.angular.z = 0.15
            self.pub.publish(string)
            rate.sleep()
        string.linear.x = 0
        string.angular.z = 0
        self.pub.publish(string)
        """
        print "initialization completed"
        #print "waiting for goal from move_base"

    def rrt_list_update(self):

        # if list is empty, run RRT
        if self.points_lst_meter == []:
            i = 0
            print "computing"
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
                if i % 1000 == 0:
                    print "computing"

            # check taget is at feasible region or not
            # if True
            print "1"
            if self.rrt.target_feasible == True:
                # if number <= 120, it means it reaches target
                if len(self.rrt.nodes) <= 90 and i <= 100000:
                    print "1-1"
                    print temp
                    try:

                        closest_node = temp[1]
                        print closest_node
                        print self.rrt.lines
                        path = find_path(closest_node, self.rrt.tgt,self.rrt.lines)
                    except Exception as e:
                        print e
                        path = [(closest_node,  self.rrt.tgt)]
                # if number > 120, it means it dosent reache target
                # stll need to be modified following
                else:
                    print "1-2"
                    try:
                        closest_node = connect_target(self.rrt)

                        self.rrt.lines.append((closest_node, self.rrt.tgt))
                        path = find_path(closest_node, self.rrt.tgt, self.rrt.lines)
                    except Exception as e:
                        print e

            # if False
            else:
                print "1-3"
                closest_node = connect_target(self.rrt)
                self.rrt.temp_target = closest_node

                path = find_path(closest_node, self.rrt.temp_target , self.rrt.lines)

            print "2"
            point_path = []
            print path
            if len(path) > 1:
                for i in range(len(path)):
                    point_path.append(path[i][0][0])
                    point_path.append(path[i][0][1])
                    if i == len(path)-1:
                        point_path.append(path[i][1][0])
                        point_path.append(path[i][1][1])
            else:
                point_path.append(path[0][0][0])
                point_path.append(path[0][0][1])
                point_path.append(path[0][1][0])
                point_path.append(path[0][1][1])

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
            if len(point_path) >1:
                for i in range(len(point_path)/2):
                    (x,y)=cell_to_meter(self.map_origin_meter, (point_path[2*i],point_path[2*i+1]), self.map_resolution )
                    points_lst.append(x)
                    points_lst.append(y)
            else:
                (x1,y1)=cell_to_meter(self.map_origin_meter, (point_path[0][0]), self.map_resolution )
                points_lst.append(x1)
                points_lst.append(y1)
                (x2,y2)=cell_to_meter(self.map_origin_meter, (point_path[0][0]), self.map_resolution )
                points_lst.append(x2)
                points_lst.append(y2)
            # points_lst is the list of waypoints in meter unit
            print points_lst, "unit: meter"
            self.points_lst_meter = points_lst
            waypoints_msg = waypoints()
            waypoints_msg.mid_points_cell= point_path
            waypoints_msg.list_in_meter = points_lst
            self.pub_rrt.publish(waypoints_msg)
            print "send rrt msg"

            self.sending_path(points_lst)
            # generate path msg
            """
            path_list_msg = []
            for i in range(len(points_lst)/2):
                path_list_msg.append((points_lst[2*i], points_lst[2*i+1]))

            poses_collection = []
            for item in path_list_msg:
                temp = pt_2_Pose(item)
                poses_collection.append(temp)

            path = sending_points()
            path.path_sender(poses_collection)
            # end of generating msg
            """


        else:
            try:
                if self.odom_name == None or self.odom_name == "odom":
                    current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                else:
                    current_pose_msg = rospy.wait_for_message(self.odom_name, Odometry)

            except:
                current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)

            """
            if self.selection_p == "0":
                current_pose_msg = rospy.wait_for_message("/odom", Odometry)
            elif self.selection_p == "1":
                current_pose_msg = rospy.wait_for_message("/camera/odom", Odometry)
            elif self.selection_p == "2":
                current_pose_msg = rospy.wait_for_message("/ORB_SLAM/scaled_odom", Odometry)
            else:
                #husky simulation
                current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
            """
            try:
                robot_x = current_pose_msg.pose.pose.position.x
                robot_y = current_pose_msg.pose.pose.position.y
            except:
                robot_x = current_pose_msg.transform.translation.x
                robot_y = current_pose_msg.transform.translation.y

            if self.rrt.target_feasible == True:

                e_x = self.target_point_x_meter - robot_x
                e_y = self.target_point_y_meter - robot_y
                dist = math.sqrt((e_x)**2+(e_y)**2)
                if dist <= 0.30:
                    #reset target (x,y)
                    print "waiting for goal from move_base"
                    goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
                    self.target_point_x_meter = goal_msg.pose.position.x
                    self.target_point_y_meter = goal_msg.pose.position.y
                    self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)

                    self.reset()

            else:
                rrt_temp_tgt_meter = cell_to_meter(self.map_origin_meter, self.rrt.temp_target, self.map_resolution)

                e_x = rrt_temp_tgt_meter[0] - robot_x
                e_y = rrt_temp_tgt_meter[1] - robot_y
                dist = math.sqrt((e_x)**2+(e_y)**2)
                if dist <= 0.25:
                    #reset target (x,y)
                    print "waiting for map updating and run RRT"
                    """
                    goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
                    self.target_point_x_meter = goal_msg.pose.position.x
                    self.target_point_y_meter = goal_msg.pose.position.y
                    self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)
                    """
                    #re-get current robot position(get msg for one time)
                    try:
                        pose_msg = rospy.wait_for_message(self.odom_name, Odometry)
                    except:
                        pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                    """
                    if self.selection_p == "0":
                        pose_msg = rospy.wait_for_message("/odom", Odometry)
                    elif self.selection_p == "1":
                        pose_msg = rospy.wait_for_message("/camera/odom", Odometry)
                    elif self.selection_p == "2":
                        pose_msg = rospy.wait_for_message("/ORB_SLAM/scaled_odom", Odometry)
                    else:
                        #husky simulation
                        pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                    """
                    try:
                        self.starting_x_meter = pose_msg.pose.pose.position.x
                        self.starting_y_meter = pose_msg.pose.pose.position.y
                        self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
                    except:
                        self.starting_x_meter = pose_msg.transform.translation.x
                        self.starting_y_meter = pose_msg.transform.translation.y
                        self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)

                    #re-get map info
                    map_info_msg = rospy.wait_for_message(self.map_name,OccupancyGrid)
                    """
                    if self.selection_m == "0":
                        map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
                    elif self.selection_m == "1":
                        map_info_msg = rospy.wait_for_message("/rtabmap/grid_map",OccupancyGrid)
                    elif self.selection_m == "2":

                        map_info_msg  = rospy.wait_for_message("/global_map", OccupancyGrid)
                    else:
                        #husky simulation
                        map_info_msg  = rospy.wait_for_message("/map", OccupancyGrid)
                    """

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

    def sending_path(self, points_lst):

        path_list_msg = []
        for i in range(len(points_lst)/2):
            path_list_msg.append((points_lst[2*i], points_lst[2*i+1]))

        poses_collection = []
        for item in path_list_msg:
            temp = pt_2_Pose(item)
            poses_collection.append(temp)

        path = sending_points()
        path.path_sender(poses_collection)


    def direct_connect(self):
        if self.points_lst_meter == []:

            print "directly connect 2 points"

            point_path = [self.starting_robot_cell[0], self.starting_robot_cell[1],
                          self.target_point_cell[0], self.target_point_cell[1]]
            points_lst = [self.starting_robot_meter[0], self.starting_robot_meter[1],
                          self.target_point_meter[0], self.target_point_meter[1]]
            waypoints_msg = waypoints()
            waypoints_msg.mid_points_cell= point_path
            waypoints_msg.list_in_meter = points_lst
            self.points_lst_meter = points_lst
            self.pub_rrt.publish(waypoints_msg)
            print waypoints_msg
            print "send msg"
            self.sending_path(points_lst)
        else:
            try:
                if self.odom_name == None or self.odom_name == "odom":
                    current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
                else:
                    current_pose_msg = rospy.wait_for_message(self.odom_name, Odometry)

            except:
                current_pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)


            try:
                robot_x = current_pose_msg.pose.pose.position.x
                robot_y = current_pose_msg.pose.pose.position.y
            except:
                robot_x = current_pose_msg.transform.translation.x
                robot_y = current_pose_msg.transform.translation.y
            e_x = self.target_point_x_meter - robot_x
            e_y = self.target_point_y_meter - robot_y
            dist = math.sqrt((e_x)**2+(e_y)**2)
            if dist <= 0.30:
                #reset target (x,y)
                print "waiting for goal from move_base"
                goal_msg = rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
                self.target_point_x_meter = goal_msg.pose.position.x
                self.target_point_y_meter = goal_msg.pose.position.y
                self.target_point_meter = (self.target_point_x_meter,self.target_point_y_meter)

                self.reset()

    def reset(self):
        #reset target (x,y)


        #re-get current robot position(get msg for one time)
        try:
            pose_msg = rospy.wait_for_message(self.odom_name, Odometry)
        except:
            pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
        """
        if self.selection_p == "0":
            pose_msg = rospy.wait_for_message("/odom", Odometry)
        elif self.selection_p == "1":
            pose_msg = rospy.wait_for_message("/camera/odom", Odometry)
        elif self.selection_p == "2":
            pose_msg = rospy.wait_for_message("/ORB_SLAM/scaled_odom", Odometry)
        else:
            #husky simulation
            pose_msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
        """

        try:
            self.starting_x_meter = pose_msg.pose.pose.position.x
            self.starting_y_meter = pose_msg.pose.pose.position.y
            self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)
        except:
            self.starting_x_meter = pose_msg.transform.translation.x
            self.starting_y_meter = pose_msg.transform.translation.y
            self.starting_robot_meter = (self.starting_x_meter, self.starting_y_meter)

        #re-get map info
        map_info_msg = rospy.wait_for_message(self.map_name,OccupancyGrid)
        """
        if self.selection_m == "0":
            map_info_msg = rospy.wait_for_message("/map",OccupancyGrid)
        elif self.selection_m == "1":
            map_info_msg = rospy.wait_for_message("/rtabmap/grid_map",OccupancyGrid)
        elif self.selection_m == "2":

            map_info_msg  = rospy.wait_for_message("/global_map", OccupancyGrid)
        else:
            #husky simulation
            map_info_msg  = rospy.wait_for_message("/map", OccupancyGrid)
        """
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

    def spin(self):
        rospy.loginfo("Start RRT")
        #rate = rospy.Rate(self.rate)
        rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            # if tracking is ok and joystick is at auto
            flag_auto = self.check_auto_com()
            flag_tracking = self.check_tracking_com()
            #print flag_auto, flag_tracking
            if flag_auto and flag_tracking:
                flag_direct = check_line(self.starting_robot_cell,
                                     self.target_point_cell,
                                     self.threshold, self.map_array)
                #print flag_direct
                if flag_direct:
                    self.direct_connect()
                    rate.sleep()
                else:
                    self.rrt_list_update()
                    rate.sleep()
            # else reset everything
            else:
                print "reset RRT"
                self.reset()
                rate.sleep()


        #rospy.spin() #indent position???

    def shutdown(self):
        rospy.loginfo("Shutting down RRT")

    def check_auto_com(self):
        """
        topic_list = rospy.get_published_topics()
        topics = []
        for topic in topic_list:
            topics.append(topic[0])
        """
        if '/husky_modeswitch/controlhardware_interface' in self.topics:
            auto_msg = rospy.wait_for_message('/husky_modeswitch/controlhardware_interface',husky_controlinfo)
            flag_auto = auto_msg.auto
        else:
            flag_auto = True

        if flag_auto is True:
            return True
        else:
            return False



    def check_tracking_com(self):
        """
        need revise tracking topic
        """
        """
        topic_list = rospy.get_published_topics()
        topics = []
        for topic in topic_list:
            topics.append(topic[0])
        """
        if '/ORB_SLAM/Track' in self.topics:
            track_msg = rospy.wait_for_message('/ORB_SLAM/Track',Int32)
            flag_tracking = track_msg
        else:
            flag_tracking = 4

        if flag_tracking is 4:
            return True
        else:
            return flag_tracking


def main():
    rrt_list = RRTList()
    #print "waiting for goal from move_base"
    rrt_list.spin()


if __name__ == '__main__':

    main()
