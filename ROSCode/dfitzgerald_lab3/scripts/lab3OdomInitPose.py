#! /usr/bin/env python

import rospy

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped


import math
import numpy
import random
import copy

# def odomCallback(data):
#     print "Received original odom"
#     updateOdom(data)

def shiftedOdomCallback(data):
    print "Received Shifted Odom"
    updateOdom(data)
    
def updateOdom(data):    
#     print "I got data!"
#     print data
#copied from the sample code i
    
    initPose = PoseWithCovarianceStamped()
    initPose.pose.pose.position.x = data.pose.pose.position.x
    initPose.pose.pose.position.y = data.pose.pose.position.y
    initPose.pose.pose.orientation = data.pose.pose.orientation

#     print "O'("+str(initPose.pose.pose.position.x)+","+str(initPose.pose.pose.position.y)+") A"+str(initPose.pose.pose.orientation.w)
    
    
    initpose_pub.publish(initPose)


if __name__ == '__main__':
    
    # TEST EVERYTHING
#     testAll()
    
    # register our node with roscore under this name
    rospy.init_node('LAB3_OdomInitPose')  
    
    ############### publishers
    rospy.loginfo("MAKING PUBLISHERS")
    initpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
    
    ############### SUBSCRIBERS
    rospy.loginfo("SUBSCRIBING TO TOPICS")
#     rospy.Subscriber("odom", Odometry, odomCallback)
    rospy.Subscriber('lab3_shifted_Odom', Odometry, shiftedOdomCallback)
    
    print "Odom to InitPoses Running..."
    
    # MAIN PROGRAM
    try:
        print "\n"*2
        rospy.sleep(0.5)  # wait for first map to come in before doing anything

        # LOOP FOREVER [or rospy.spin()]
        while not rospy.is_shutdown():
            rospy.spin()  
    except rospy.ROSInterruptException:  # main program loops so check it for exceptions while its waiting inbetween loops
        pass
