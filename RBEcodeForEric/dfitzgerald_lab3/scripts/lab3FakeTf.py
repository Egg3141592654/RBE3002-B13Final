#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage

mapOffset = Transform()
gotMap = 0

def TFCallback(data):
#     print "Received transform"
    global mapOffset
    for TransformStamped in data.transforms: 
        if TransformStamped.header.frame_id == "map" and TransformStamped.child_frame_id == "odom":
            mapOffset = TransformStamped.transform
            gotMap = 1
            
def callback(data):
#     print "Received odom..."
    global mapOffset
    global odomPub
    
    #make odometry offset from the original odometry by the mapOffset
    Odom = Odometry()
    Odom.pose.pose.position.x = data.pose.pose.position.x + mapOffset.translation.x
    Odom.pose.pose.position.y = data.pose.pose.position.y + mapOffset.translation.y
    Odom.pose.pose.position.z = data.pose.pose.position.z + mapOffset.translation.z
    
#     print "O'("+str(Odom.pose.pose.position.x)+","+str(Odom.pose.pose.position.y)+")"
    
#     if (Odom.pose.pose.position.x <0 or Odom.pose.pose.position.y <0):
#         print Odom.pose.pose.position
#         raise Exception("NEGATIVE COORDINATES! ")
    
    #set the orientation based on the quaternion from the transformed position
    quat = data.pose.pose.orientation
    q = [quat.x,quat.y,quat.z,quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    quat = mapOffset.rotation
    q = [quat.x,quat.y,quat.z,quat.w]
    rollOff, pitchOff, yawOff = euler_from_quaternion(q)
    roll += rollOff
    pitch += pitchOff
    yaw += yawOff
    quater = quaternion_from_euler(roll, pitch, yaw)
    Odom.pose.pose.orientation.w = quater[3]
    Odom.pose.pose.orientation.x = quater[0]
    Odom.pose.pose.orientation.y = quater[1]
    Odom.pose.pose.orientation.z = quater[2]
    
#     print "Publishing shifted odom...\n"
    odomPub.publish(Odom)
    
    

if __name__ == '__main__':
    print "Transform Spoofer Running"
    rospy.init_node('lab3_fakeTF')
    odomPub = rospy.Publisher('lab3_shifted_Odom', Odometry)

    rospy.Subscriber("odom", Odometry, callback)
    rospy.Subscriber("/tf", TFMessage, TFCallback)


    while not rospy.is_shutdown():
        rospy.spin()