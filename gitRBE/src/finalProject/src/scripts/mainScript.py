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
        toSend.pose.orientation.x = x
        toSend.pose.orientation.y = y
        toSend.pose.orientation.z = 0
        toSend.pose.orientation.w = theta
        
        self.sub.publish(toSend)

if name=="__name__":
    #the hard set goals we will try to go to will be set as such
    
    #Search in a pattern around the map. This is going to be something that 
    #I think that a Z would be the most appropriate.
    
    #while moving to the positions, identify people and clusters.