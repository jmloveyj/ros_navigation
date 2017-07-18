#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
# from binascii import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import tf

global vel_pub


def commandVel(linVel, angVel):
    global vel_pub
    vel = Twist()
    vel.linear.x = linVel
    vel.angular.z = angVel
    vel_pub.publish(vel)


class TurnToStart:

    def __init__(self):
        self.ifDone = False
        rospy.loginfo("start")
        self.rangeSubscriber = rospy.Subscriber(
            "/front_range", Range, self.onReceiveRange)
        self.odomSubscriber = rospy.Subscriber(
            "/odom", Odometry, self.onReceiveOdom)
        self.RANAE_THRESHOLD = 0.3
        self.ifCatchObstacle = False
        self.ifInRange = False
        self.ifLastInRange = False
        self.startYaw = 0
        self.endYaw = 0

    def onReceiveRange(self, msg):
        range = msg.range
        if(range <= self.RANAE_THRESHOLD):
            if(self.ifLastInRange):
                self.ifInRange = True
            self.ifLastInRange = True
        else:
            if(not self.ifLastInRange):
                self.ifInRange = False
            self.ifLastInRange = False

    def onReceiveOdom(self, msg):
        if(self.ifDone):
            return
        if(self.ifCatchObstacle):
            if(not self.ifInRange):
                commandVel(0, 0)
                self.ifDone = True
                (r, p, self.endYaw) = tf.transformations.euler_from_quaternion(
                    [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

                rospy.loginfo("end edge: %f", self.endYaw)
                rospy.loginfo("end edge, the angular is %f",
                              math.degrees(math.fabs(self.endYaw - self.startYaw)%3.1415))
                return
        else:
            if(self.ifInRange):
                self.ifCatchObstacle = True
                (r, p, self.startYaw) = tf.transformations.euler_from_quaternion(
                    [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                rospy.loginfo("Start to move to measure %f", self.startYaw)
        commandVel(0.0, math.radians(-15))

    def waitForFinshed(self):
        while not self.ifDone:
            rospy.sleep(2)

        self.rangeSubscriber.unregister()
        self.odomSubscriber.unregister()


if __name__ == '__main__':
        # Give the node a name
    rospy.init_node('verifyShape', anonymous=False)
    # Set rospy to execute a shutdown function when exiting

    global vel_pub
    vel_pub = rospy.Publisher(
        "/mobile_base/commands/velocity", Twist, queue_size=10)

    processer = TurnToStart()
    processer.waitForFinshed()

    rospy.loginfo("Done")
