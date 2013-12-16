#! /usr/bin/env python

import rospy

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf.transformations

import math
import numpy
import random
import copy

print ("\n"*50) + "LAB3 DRIVER SCRIPT STARTED..."

curVals = {   #initialize curVals (odometry)
           "px":0,
           "py":0,
           "theta":0,
           "vx":0,
           "vy":0,
           "vtheta":0
           }

G_Path=[]    #global path to try to follow
G_CurPosition=Point()
G_CurrentTheta=0
G_PathChanged=False
G_SleepTime=0.1

# def odomCallback(data):
#     updateOdom(data)

def shiftedOdomCallback(data):
    print "Shifted Odom Received"
    updateOdom(data)
    
def updateOdom(data):    
#     print "I got data!"
#     print data
#copied from the sample code i
    global curVals
    
    
    
    curVals["px"] = data.pose.pose.position.x
    curVals["py"] = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    curVals["theta"] = math.degrees(yaw) + 180
    curVals["vx"] = data.twist.twist.linear.x
    curVals["vy"] = data.twist.twist.linear.y
    curVals["vtheta"] = data.twist.twist.angular.z
    
    #set as current Position and orientation
    global G_CurPosition
    global G_CurrentTheta
    G_CurPosition.x=curVals["px"]
    G_CurPosition.y=curVals["py"]
    G_CurrentTheta=curVals["theta"]
    
    print "O'("+str(G_CurPosition.x)+","+str(G_CurPosition.y)+") A"+str(G_CurrentTheta)
    
def publishTwist(u,w):
    twist = Twist()
#     rospy.loginfo(twist)
    twist.linear.x = u
    twist.angular.z = w
    twistPublisher.publish(twist)
#     rospy.sleep(0.1)

def spinWheels(u1, u2, time):
    #use the velocity kinematics to get u and w from the wheel speeds
    u = 1.9*(u1+u2)   
    w = .165*(u1-u2)
    publishTwist(u,w) #publish twist message for that u and w
    desiredTime = rospy.get_time() + time #the desired time is the current time + the new time
    
    while (rospy.get_time() != desiredTime): #wait until that time is reached
        rospy.get_time()
        publishTwist(u,w) #keep publishing until then
    
def isXClose(current,desired):
    #return true if the current x is within .1 of the desired x
    if (current < (desired + .1)) and (current > (desired - .1)):
        return True
    
def isYClose(desired):
    #return true if the current y is within .1 of the desired y
    if ((curVals["py"] < (desired + .1)) and (curVals["py"] > (desired - .1))):
        return True
     
def driveStraight(speed,distance):
    print "DRIVING STRAIGHT..."
    
    u = speed             #driving straight so both wheels go same speed
    t = distance/speed    #time to drive is desired distance/ desired speed
    
    initialX = curVals["px"] #get the initial x position
    initialY = curVals["py"] #get the initial y position
    curDist = math.sqrt(initialY*initialY + initialX*initialX) #extrapolate the current distance from x and y positions
    
    endDist = curDist + distance  #set desired end distance to current distance + new distance
    print "currentDistance = " + str(curDist)
    print "distance = " + str(distance)
    print "end distance = " + str(endDist)
    print
    
    
    while (not isXClose(curDist,distance) and (not G_PathChanged)): #while the distance is not within margin of error of desired distance
        curX = curVals["px"] - initialX  #get the x
        curY = curVals["py"] - initialY  #get the y
        curDist =  math.sqrt(curX**2 + curY**2) #get the distance to compare next time the loop iterates
        publishTwist(speed,0) #keep drivign straight
#         print "current distance:" + str(curDist)+"\tdistance to go= " + str(distance)+"\tend distance = " + str(endDist)
        print "d" + str(curDist)+"\t+" + str(distance)+"\t=" + str(endDist)
        rospy.sleep(G_SleepTime)
    
def isAngleClose(desiredTheta):
    # returns true if the current angle is within 5 of the desired angle
#     print "desired Theta: ", desiredTheta
#     print "Actual Theta;  ", curVals["theta"]
    return ((curVals["theta"] < (desiredTheta + 5)) and (curVals["theta"] > (desiredTheta - 5)))
    
def rotate(angle):
    print "\tRotating..."
    speed = 0.5
    
#     print "THETA"
#     print curVals["theta"]
#     print
    
    desTheta = curVals["theta"]+angle
    if desTheta > 360:
        desTheta = desTheta - 360
    elif desTheta < 0:
        desTheta = (360+desTheta)
        
    while (not isAngleClose(desTheta)) and (not G_PathChanged):
        print "\tA:"+str(curVals["theta"])+"-"+str(desTheta)
        rospy.sleep(G_SleepTime)
        if angle >= 0:                     #if angle is positive
            publishTwist(0,speed)  #turn left
        else:                   #if angle is negative
            publishTwist(0,-speed)  #turn right

def driveToPoint(point):

    y2 = G_CurPosition.y
    x2 = G_CurPosition.x
    
    x1 = point.x
    y1 = point.y
        
    print "startpoint = (" + str(x1) + "," + str(y1) + ")"
    print "nextpoint = (" + str(x2) + "," + str(y2) + ")"
        
    desiredTheta = math.atan2((y2 - y1), (x2 - x1))
    desiredTheta = math.degrees(desiredTheta)
        
    print "desiredtheta = " + str(desiredTheta)
    # set current theta equal to odometry reading
    currentTheta = G_CurrentTheta
    print "currenttheta = " + str(G_CurrentTheta)
    angleToTurn = desiredTheta - currentTheta
    print "angleToTurn = " + str(angleToTurn)
        
    if (angleToTurn < 0):
        angleToTurn = angleToTurn + 360
    print "Angle to Turn: " + str(angleToTurn)

    if (angleToTurn > 180):
        angleToTurn = (360 - angleToTurn) * (-1)
#         
    distance = math.sqrt(math.pow((y2 - y1), 2) + math.pow((x2 - x1), 2))
#         distanceInMeters = distance*0.05
        

    
    #execute motions!
    print "EXECUTING MOTION..."
    rotate(angleToTurn)
    print "Distance to Drive: "+str(distance)
    driveStraight(.1, distance) 
    print "...DONE EXECUTING MOTION"
        
def driveAlongPath():
    global G_Path
    global G_PathChanged
    while(G_Path):
        driveToPoint(G_Path.pop())
        G_PathChanged=False

def newPathCallback(data):
    "RECEIVED NEW PATH"
    
    pathPoints= [poseStamped.pose.position for poseStamped in data.poses]
    print "\tReceived "+str(len(pathPoints))+" points in path:"
    for point in pathPoints:
        print "\tP("+str(point.x)+","+str(point.y)+")"
    
    #set global path as path (NOTE IT IS STILL IN REVERSE ORDER!)
    global G_Path
    global G_PathChanged
    G_Path=pathPoints
    G_PathChanged=True

if __name__ == '__main__':
    
    # TEST EVERYTHING
#     testAll()
    
    # register our node with roscore under this name
    rospy.init_node('LAB3_Driver')  
    
    ############### publishers
    rospy.loginfo("MAKING PUBLISHERS")
    twistPublisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)

    ############### SUBSCRIBERS
    rospy.loginfo("SUBSCRIBING TO TOPICS")
    rospy.Subscriber('lab3_path', Path, newPathCallback)
#     rospy.Subscriber("odom", Odometry, odomCallback)
    rospy.Subscriber('lab3_shifted_Odom', Odometry, shiftedOdomCallback)
    
    # MAIN PROGRAM
    try:
        print "\n"*2
        rospy.sleep(0.5)  # wait for first map to come in before doing anything

        # LOOP FOREVER [or rospy.spin()]
        while not rospy.is_shutdown():
            driveAlongPath()  
    except rospy.ROSInterruptException:  # main program loops so check it for exceptions while its waiting inbetween loops
        pass
