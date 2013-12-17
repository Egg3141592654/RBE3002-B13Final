import rospy

from geometry_msgs.msg import PoseStamped

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
        toSend = PoseStamped()
        toSend.header.frame_id = "map"
        toSend.pose.position.x = x
        toSend.pose.position.y = y
        toSend.pose.position.z = 0
        toSend.pose.orientation.w = theta
        
        self.sub.publish(toSend)
        
    def isCloseEnough(self, currX, currY, currW):
        """
        This method determines if we are close enough to the goal to be 
        """
        
    

if name=="__name__":
    setter = GoalPosition()
    
    #the hard set goals we will try to go to will be set as such
    point1X = -.695
    point1Y = 1.393
    point1W = .80825
    
    point2X = 2.487
    point2Y = 4.308
    point2W = .65865
    
    point3X = 1.579
    point3Y = -.5466
    point3W = .91519
    
    point4X = 4.7899
    point4Y = 1.909
    point4W = .05823
    
    #Search in a pattern around the map. This is going to be something that 
    #I think that a Z would be the most appropriate.
    
    #while moving to the positions, identify people and clusters.