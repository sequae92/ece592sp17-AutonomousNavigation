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

class OnePoint:
    def __init__(self, x,y):
        rospy.init_node('Location_info',  anonymous=True)
        self.rate = 5
        self.x_tgt = x
        self.y_tgt = y
        self.start_time = rospy.Time.now()
        self.x = -200
        self.y = -200
        self.theta = 0
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
        #print self.x, self.y,self.theta

    #could delete pose_update???
    def pose_update(self):
        rospy.Subscriber('/husky_location_tf', TransformStamped, self.pose )
        #print self.x, self.y,self.theta
    def CmdVel_update(self):
        e_x     = self.x_tgt - self.x
        e_y     = self.y_tgt - self.y
        path_angle = math.atan2(e_y,e_x)
        e_theta_path = e_angle_handler(path_angle, self.theta)
        string = Twist()

        dist = math.sqrt((e_x)**2+(e_y)**2)
        if dist > 0.05:

            if abs(e_theta_path) > 0.1:
                w = e_theta_path
                w = min(w, 0.2)
                string.linear.x = 0
                string.angular.z = w
                print "turning"

            else:
                x = dist
                x = min(0.8,x)
                w = e_theta_path
                string.linear.x = x
                string.angular.z = w
                print "heading"
            print self.x, self.y
        else:
            print "reach target"
            print "enter target point and orientation, 2 values seperated by space(unit:m):"

            s = raw_input().split(" ")
            target =  (float(s[0]), float(s[1]))
            self.x_tgt, self.y_tgt = target

        self.pub.publish(string)


    def spin(self):
        rospy.loginfo("Start gopigo_controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            #self.pose_update()
            self.CmdVel_update()
            #
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down one point navigation")
        # send commmand = 0
        self.pub.publish(Twist())

def main(x_tgt, y_tgt):
    one_point = OnePoint(x_tgt, y_tgt)
    one_point.spin()

if __name__ == '__main__':
    print "enter target point, 2 values seperated by space(unit:m):"
    s = raw_input().split(" ")
    target =  (float(s[0]), float(s[1]))

    x_tgt, y_tgt = target
    main(x_tgt, y_tgt)
