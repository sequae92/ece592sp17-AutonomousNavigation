#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32, Int32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from husky_modeswitch.msg import husky_controlinfo
import tf2_ros
from rrt_navigation.msg import waypoints
from path_following.msg import check_path
from tf.transformations import euler_from_quaternion

def e_angle_handler(theta_t, theta_c):
    if theta_t >= 0 and theta_c >= 0:
        e_init = theta_t - theta_c
        return e_init
    elif theta_t <= 0 and theta_c <= 0:
        e_init = theta_t - theta_c
        return e_init
    elif theta_t >= 0 and theta_c <= 0:
        if theta_c <= theta_t- math.pi:
            e_init = theta_t - (theta_c+2*math.pi)
            return e_init
        else:
            e_init = theta_t - theta_c
            return e_init
    else:
        if theta_c <= theta_t + math.pi:
            e_init = theta_t - theta_c
            return e_init
        else:
            e_init = theta_t - (theta_c-2*math.pi)
            return e_init
def array_to_pair(data):
    temp = []
    for i in range(len(data)/2):
        temp.append([data[2*i], data[2*i+1]])
    return temp

class MultiPoint:
    def __init__(self):
        rospy.init_node('path_following',  anonymous=True)

        topic_list = rospy.get_published_topics()
        self.topics = []
        for topic in topic_list:
            self.topics.append(topic[0])

        self.rate = 5
        if '/husky_modeswitch/controlhardware_interface' in self.topics:
            #for real world
            print "real world"
            self.pub = rospy.Publisher('/husky_controller/velocitycontrol',Twist ,queue_size=5)
        else:
            #for real world
            self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist,queue_size=5)



        #save Twist msg for reverse movement
        self.x_vel = [0]*120
        self.z_ang_vel = [0]*120

        # update self.x, self.y, self.theta
        try:
            self.odom_name = rospy.get_param('odom_topic')
        except:
            self.odom_name = '/husky_location_tf'
        #robot current position
        self.pose()


        self.pub_path_check =  rospy.Publisher('/send_current_path', check_path, queue_size=5, latch = True)

        print "all set, wait RRT msg"


        points_msg = rospy.wait_for_message('/way_point_info', waypoints)
        print "got it"
        self.data = array_to_pair(points_msg.list_in_meter)
        # publish following
        self.data_cell = points_msg.mid_points_cell
        self.length = len(self.data)
        #the final goal
        self.x_tgt_final, self.y_tgt_final = self.data[-1]
        #the current goal
        self.x_tgt_curr, self.y_tgt_curr = self.data[0]
        #record how many waypoint had been achieved
        self.count = 0

    # get current pose, rotation
    def pose(self):

        #print msg
        try:

            msg = rospy.wait_for_message(self.odom_name, Odometry, timeout = 0.1)
            if self.odom_name == None or self.odom_name == "odom" or msg == None:
                msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)
        except:
            msg = rospy.wait_for_message('/husky_location_tf', TransformStamped)

        try:

            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q_1 = msg.pose.pose.orientation.x
            q_2 = msg.pose.pose.orientation.y
            q_3 = msg.pose.pose.orientation.z
            q_0 = msg.pose.pose.orientation.w
            quaternion =(q_1,q_2, q_3, q_0)
            (roll,pitch,yaw) = euler_from_quaternion(quaternion)
            self.theta = yaw
        except:

            self.x = msg.transform.translation.x
            self.y = msg.transform.translation.y
            q_1 = msg.transform.rotation.x
            q_2 = msg.transform.rotation.y
            q_3 = msg.transform.rotation.z
            q_0 = msg.transform.rotation.w
            quaternion =(q_1,q_2, q_3, q_0)
            (roll,pitch,yaw) = euler_from_quaternion(quaternion)
            self.theta = yaw

    def reverse_motion(self):
        string = Twist()
        n = len(self.x_vel)
        rate = rospy.Rate(self.rate)
        #print self.x_vel
        #print self.z_ang_vel
        for i in range(n):
            print i, "comd"
            string.linear.x = -1*self.x_vel[i]
            string.angular.z = -1*self.z_ang_vel[i]
            self.pub.publish(string)
            rate.sleep()
        string.linear.x = 0
        string.angular.z = 0
        self.pub.publish(string)

    def CmdVel_update(self):

        e_x     = self.x_tgt_curr - self.x
        e_y     = self.y_tgt_curr - self.y
        path_angle = math.atan2(e_y,e_x)
        e_theta_path = e_angle_handler(path_angle, self.theta)
        string = Twist()
        print "e_x, e_y, e_theta"
        print e_x, e_y, e_theta_path
        dist = math.sqrt((e_x)**2+(e_y)**2)
        if dist > 0.20:
            if abs(e_theta_path) > 0.1:
                w = e_theta_path
                #print w
                if w > 0:
                    w = min(w, 0.08)
                else:
                    w = max(w, -0.08)
                string.linear.x = 0
                string.angular.z = w
                self.x_vel.pop()
                self.x_vel.insert(0,0)
                self.z_ang_vel.pop()
                self.z_ang_vel.insert(0,w)
                print "turning"

            else:
                x = dist
                x = min(0.08,x)
                w = e_theta_path
                if w > 0:
                    w = min(w, 0.08)
                else:
                    w = max(w, -0.08)
                string.linear.x = x
                string.angular.z = w/4
                self.x_vel.pop()
                self.x_vel.insert(0,x)
                self.z_ang_vel.pop()
                self.z_ang_vel.insert(0,w)
                print "heading"
            #print self.x, self.y
        else:
            if self.count < self.length-1 :
                self.count = self.count +1
                print "reach %d waypoint" %self.count
                self.x_tgt_curr, self.y_tgt_curr =self.data[self.count]
            else:
                # reach final goal, reset
                self.reset()
        print "Vel_linear, Vel_angular"
        print string.linear.x, string.angular.z
        self.pub.publish(string)
        # send msg of path information
        path_info = check_path()
        path_info.passed = self.count
        path_info.mid_points_cell = self.data_cell
        self.pub_path_check.publish(path_info)


    def reset(self):
        string = Twist()
        self.pub.publish(string)

        print "waiting for another RRT output"
        points_msg = rospy.wait_for_message('/way_point_info', waypoints)
        self.data = array_to_pair(points_msg.list_in_meter)
        self.data_cell = points_msg.mid_points_cell
        self.length = len(self.data)
        #the final goal
        self.x_tgt_final, self.y_tgt_final = self.data[-1]
        #the current goal
        self.x_tgt_curr, self.y_tgt_curr = self.data[0]
        #record how many waypoint had been achieved
        self.count = 0
        #robot fake current position
        self.x = -200
        self.y = -200
        self.theta = 0
        # update self.x, self.y, self.theta
        self.pose()



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

    def spin(self):
        rospy.loginfo("Start controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.check_auto_com() and self.check_tracking_com():
                self.pose()
                #rospy.Subscriber('/husky_location_tf', TransformStamped, self.pose )
                self.CmdVel_update()

            elif  self.check_auto_com() and self.check_tracking_com() is 3:
                print "reverse motion"
                #how to comunicate with rrt
                #reset??
                self.reverse_motion()
            else:
                print "reset"
                string = Twist()
                self.pub.publish(string)
                self.reset()

            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down list points navigation")

        # send commmand = 0
        self.pub.publish(Twist())


def listConverter(s):
    temp = []
    for i in range(len(s)/2):
        temp.append([float(s[2*i]), float(s[2*i+1])])
    return temp


def main():
    multi_point = MultiPoint()
    multi_point.spin()

if __name__ == '__main__':

    main()
