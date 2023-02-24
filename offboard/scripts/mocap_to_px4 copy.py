#!/usr/bin/env python
#Node converts the mocap positions to the px4 NED notation
import rospy
from geometry_msgs.msg import PoseStamped

mocap_pose=PoseStamped()

def mocap_to_px4():
    #start node for the motion capture default pose data
    rospy.init_node("mocap_to_px4",anonymous=True)
    #print("Here2")
    #subscribe to the mocap message Optitrack
    rospy.Subscriber("Rickbot/pose",PoseStamped,px4_pose_callback,queue_size=1)
    pub=rospy.Publisher("mavros/vision_pose/pose",PoseStamped,queue_size=1)
    rate=rospy.Rate(100)#delay 50
    while not rospy.is_shutdown():
        pub.publish(px4_pose())
        rate.sleep()        

def px4_pose_callback(mocap_pose_tmp):
    global mocap_pose 
    mocap_pose=mocap_pose_tmp

def px4_pose():
    #Aligned the PoseStamped message from the mocap Y-up frame
    global mocap_pose
    pose=PoseStamped()
    #Match id of header
    pose.header=mocap_pose.header
    #Aligned the PoseStamped message from the mocap Y-up frame (not necessary)
    pose.pose.position.x=mocap_pose.pose.position.x
    pose.pose.position.y=mocap_pose.pose.position.y
    pose.pose.position.z=mocap_pose.pose.position.z
    pose.pose.orientation.x=mocap_pose.pose.orientation.x
    pose.pose.orientation.y=mocap_pose.pose.orientation.y
    pose.pose.orientation.z=mocap_pose.pose.orientation.z
    pose.pose.orientation.w=mocap_pose.pose.orientation.w
    return pose
    
if __name__=='__main__':
    try:     
        mocap_to_px4()
    except rospy.ROSInternalException:
        pass



    


