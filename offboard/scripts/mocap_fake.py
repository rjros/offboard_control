#!/usr/bin/env python3
#Node converts the mocap positions to the px4 NED notation
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import rotation_matrix
from tf.transformations import concatenate_matrices
from mavros import setpoint as SP
from std_msgs.msg import Header

mocap_pose=PoseStamped()

def mocap_to_px4():
    stamp=0
    #start node for the motion capture default pose data
    rospy.init_node("mocap_to_px4",anonymous=True)
    #print("Here2")
    #subscribe to the mocap message Optitrack
    # rospy.Subscriber("Rickbot/pose",PoseStamped,px4_pose_callback,queue_size=1)
    pub=rospy.Publisher("mavros/vision_pose/pose",PoseStamped,queue_size=1)
    rate=rospy.Rate(100)#delay 50
    while not rospy.is_shutdown():
        pub.publish(px4_pose(stamp))
        stamp=rospy.Time.now()
        rate.sleep()        

def px4_pose_callback(mocap_pose_tmp):
    global mocap_pose 
    mocap_pose=mocap_pose_tmp

def px4_pose(stamp):
    #Aligned the PoseStamped message from the mocap Y-up frame
    global mocap_pose
    pose=PoseStamped()
    #Match id of header
    pose.header=Header()
    pose.header.stamp=stamp
    pose.header.frame_id="base_foorprint"
    pose.pose.orientation= Quaternion(*quaternion_from_euler(0,0,0))
    #Aligned the PoseStamped message from the mocap Y-up frame (not necessary)
    pose.pose.position.x=0.0
    pose.pose.position.y=0.0
    pose.pose.position.z=0.0
    return pose
    
if __name__=='__main__':
    try:     
        mocap_to_px4()
    except rospy.ROSInternalException:
        pass



    


