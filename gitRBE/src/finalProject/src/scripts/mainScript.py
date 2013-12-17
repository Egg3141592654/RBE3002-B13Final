#! /usr/bin/env python

import rospy
import pdb

import guestStorage

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

G_curxpos = 0
G_curypos = 0

def initialPoseCallback(PoseWithCovarianceStamped):
    global G_curxpos
    global G_curypos
    
    G_curxpos = PoseWithCovarianceStamped.pose.pose.position.x
    G_curypos = PoseWithCovarianceStamped.pose.pose.position.y

class GoalPosition:
    def __init__(self):
        """
        Construct an object with the initial position from the map stored inside
        """
        self.x = 0
        self.y = 0
        self.z = 0
        self.theta= 0
        self.sub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    
    def setGoal(self, x, y, theta):
        """
        This is a callback to get the initial data for the A* algorithm.
        All this does is decompose the PoseWithCovarienceStamped message into x, y, z and theta values
        """
        print "Setting Goal"
        toSend = PoseStamped()
        toSend.header.frame_id = "map"
        toSend.pose.position.x = x
        toSend.pose.position.y = y
        toSend.pose.position.z = 0
        toSend.pose.orientation.w = theta
        
        pdb.set_trace()
        print toSend
        self.sub.publish(toSend)
        
        
    def isCloseEnough(self):
        """
        This method determines if we are close enough to the goal to be 
        """
        def isXClose():
           if ((self.x - 2) > G_curxpos) and ((self.x + 2) < G_curxpos):
               return True
           else:
               return False
            
        def isYClose():
            if ((self.y - 2) > G_curypos) and ((self.y + 2) < G_curypos):
                return True
            else: 
                return False
            
        while not(isXClose() and isYClose()):
            print "Not Close Enough Yet"
            rospy.sleep(.001)
        
        print "I'm Close Enough"
        
    

if __name__ == '__main__':
    
    rospy.init_node('mainScript')
    
    setter = GoalPosition()
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseCallback)
    
    try:
        #the hard set goals we will try to go to will be set as such
        point1X = -.695
        point1Y = 1.393
        point1W = .80825
        setter.setGoal(point1X,point1Y,point1W)
        setter.isCloseEnough()
        
        point2X = 2.487
        point2Y = 4.308
        point2W = .65865
        setter.setGoal(point2X,point2Y,point2W)
        setter.isCloseEnough()
        
        point3X = 1.579
        point3Y = -.5466
        point3W = .91519
        setter.setGoal(point3X,point3Y,point3W)
        setter.isCloseEnough()
        
        point4X = 4.7899
        point4Y = 1.909
        point4W = .05823
        setter.setGoal(point4X,point4Y,point4W)
        setter.isCloseEnough()
        
        
        #Search in a pattern around the map. This is going to be something that 
        #I think that a Z would be the most appropriate.
        
        #while moving to the positions, identify people and clusters.
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
