#!/usr/bin/env python3
#Node converts the aruco position to PX4 notation
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from math import pi
import numpy as np

angles=np.array([0,0,0])
#Transformed position wrt to the UAV
aruco_pose=PoseStamped()

def aruco_to_mavros():
    global angle
    rospy.init_node("aruco_to_mocap",anonymous=True)
    #subscribe to aruco pose topic
    rospy.Subscriber("/aruco_single/pose",PoseStamped,aruco_pose_callback,queue_size=1)
    pub=rospy.Publisher("target_pose",PoseStamped,queue_size=1)

    rate=rospy.Rate(100)#delay 50
    while not rospy.is_shutdown():
        pub.publish(mavros_pose())
        # print(angles)
        rate.sleep()        

def aruco_pose_callback(aruco_pose_tmp):
    global aruco_pose 
    aruco_pose=aruco_pose_tmp

def mavros_pose():
    global arco_pose
    global angles,position
    quartenion=np.array([0,0,0,0])

    pose=PoseStamped()
    #Match id of header
    pose.header=aruco_pose.header
    #Aligned the  aruco frame to the mavros NED convention
    # pose.pose.position.x=aruco_pose.pose.position.x
    # pose.pose.position.y=aruco_pose.pose.position.y
    # pose.pose.position.z=aruco_pose.pose.position.z
    # pose.pose.orientation.x=aruco_pose.pose.orientation.x
    # pose.pose.orientation.y=aruco_pose.pose.orientation.y
    # pose.pose.orientation.z=aruco_pose.pose.orientation.z
    # pose.pose.orientation.w=aruco_pose.pose.orientation.w

    pose.pose.position.x=aruco_pose.pose.position.y
    pose.pose.position.y=-aruco_pose.pose.position.x
    pose.pose.position.z=aruco_pose.pose.position.z
    pose.pose.orientation.x=aruco_pose.pose.orientation.y
    pose.pose.orientation.y=-aruco_pose.pose.orientation.x
    pose.pose.orientation.z=-aruco_pose.pose.orientation.z
    pose.pose.orientation.w=aruco_pose.pose.orientation.w

    quartenion=[pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
    angles=euler_from_quaternion(quartenion)

    angles=(np.array(angles)*-180/pi)

    if angles[2] <0:
        angles[2]+= 180
    else:
        angles[2]-= 180
    
    pose.pose.orientation.x=angles[0]
    pose.pose.orientation.y=angles[1]
    pose.pose.orientation.z=angles[2]
    pose.pose.orientation.w=0

    return pose
    
if __name__=='__main__':
    try:     
        aruco_to_mavros()
    except rospy.ROSInternalException:
        pass



    


