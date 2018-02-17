#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import message_filters
import tf2_ros
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

class MultiPoint:
    def __init__(self, points):
        rospy.init_node('Location_info',  anonymous=True)
        self.rate = 5
        self.data = points
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
        #save Twist msg for reverse movement
        self.x_vel = [0]*120
        self.z_ang_vel = [0]*120

        # update self.x, self.y, self.theta
        rospy.Subscriber('/husky_location_tf', TransformStamped, self.pose )
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist)

    # get current pose, rotation
    def pose(self,msg):
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

    def CmdVel_update(self):
        e_x     = self.x_tgt_curr - self.x
        e_y     = self.y_tgt_curr - self.y
        path_angle = math.atan2(e_y,e_x)
        e_theta_path = e_angle_handler(path_angle, self.theta)
        string = Twist()

        dist = math.sqrt((e_x)**2+(e_y)**2)
        if dist > 0.10:
            if abs(e_theta_path) > 0.1:
                w = e_theta_path
                w = min(w, 0.2)
                string.linear.x = 0
                string.angular.z = w
                self.x_vel.pop()
                self.x_vel.insert(0,0)
                self.z_ang_vel.pop()
                self.z_ang_vel.insert(0,w)
                print "turning"

            else:
                x = dist
                x = min(0.8,x)
                w = e_theta_path
                string.linear.x = x
                string.angular.z = w
                self.x_vel.pop()
                self.x_vel.insert(0,x)
                self.z_ang_vel.pop()
                self.z_ang_vel.insert(0,w)
                print "heading"
            print self.x, self.y
        else:
            if self.count < self.length-1 :
                self.count = self.count +1
                print "reach %d waypoint" %self.count
                self.x_tgt_curr, self.y_tgt_curr =self.data[self.count]
            else:
                # reach final goal, reset
                print "enter list of way-points[x,y] seperated by space(unit:m): or r"
                s = raw_input().split(" ")
                if s[0] == "r":
                    print "call reverse"
                    self.reverse_motion()
                    print "enter list of way-points[x,y] seperated by space(unit:m): "
                    s = raw_input().split(" ")
                while len(s)%2 != 0:
                    print "the element of list must be a even number,seperated by space(unit:m):"
                    s = raw_input().split(" ")
                new_points = listConverter(s)
                self.data = new_points
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
                rospy.Subscriber('/husky_location_tf', TransformStamped, self.pose )


        self.pub.publish(string)

    def spin(self):
        rospy.loginfo("Start controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            #self.pose_update()

            self.CmdVel_update()
            #
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


def main(points):
    multi_point = MultiPoint(points)
    multi_point.spin()

if __name__ == '__main__':
    print "enter list of way-points[x,y] seperated by space(unit:m):"
    s = raw_input().split(" ")
    while len(s)%2 != 0:
        print "the element of list must be a even number,seperated by space(unit:m):"
        s = raw_input().split(" ")
    points = listConverter(s)
    main(points)
