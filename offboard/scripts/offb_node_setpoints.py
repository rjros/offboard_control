#!/usr/bin/env python3
#read current value of the camera and move in the desired direction
#read the odometry data and use that as the inital position when the mode is shift 


import time
from math import *

import rospy
import _thread as thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


import mavros
from mavros import setpoint as SP
from mavros.utils import *
from geometry_msgs.msg import PoseStamped

import numpy as np

###Value of pose

target=PoseStamped()

####Value received by the T camera 
from nav_msgs.msg import Odometry


# Params
proximity = 0.1 # metres

# Variable
setpoints=np.array([0,0,0,0])


FirstTarget = True
otx, oty, otz, oth = (0.0,0.0,0.0,0.0)
tx, ty, tz, th = (0.0,0.0,0.0,0.0)
cx, cy, cz, ch = (0.0,0.0,0.0,0.0)
hx, hy, hz=(0.0,0.0,0.0)

reached = True
moving = False

def posUpdate(msg):
    global reached
    global state
    global hx,hy,hz
    global proximity
    global otx, oty, otz

    hx = msg.pose.position.x
    hy = msg.pose.position.y
    hz = msg.pose.position.z
    dist = sqrt(pow(hx-otx, 2) + pow(hy-oty, 2) + pow(hz-otz, 2))
    # reached = (dist <= proximity)
    # reached = True

def follow_point():
    rospy.init_node('set_local_setpoint', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, posUpdate, queue_size=1)
    
    pub = SP.get_pub_position_local(queue_size=1)
    rospy.Subscriber('target_pose',PoseStamped,target_callback,queue_size=1)
    rate=rospy.Rate(10)#delay 
    while not rospy.is_shutdown():
        pub.publish(Command())
        # print(angles)
        rate.sleep()        

  

def target_callback(aruco):
    global target
    global setpoints
    target=aruco
    target_x=target.pose.position.x
    target_y=target.pose.position.y
    target_z=target.pose.position.z
    ##The orientation for the target is [roll,pitch,yaw,yaw] in eulers
    target_yaw=target.pose.orientation.z
    setpoints=[target_x,target_y,target_z,target_yaw]
    # print(setpoints)


def Command():
    global hx,hy,hz
    #print(hx,hy,hz)
    global setpoints
    msg = SP.PoseStamped(
        header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
            stamp=rospy.Time.now()),  # stamp should update
    )
    ##camera position (0.08,0.0,0.11)
    msg.pose.position.x = -hx-(setpoints[1])
    msg.pose.position.y = hy-(setpoints[0]-0.08)
    # print(hx,setpoints[0],msg.pose.position.x)
    # msg.pose.position.z = setpoints[2]
    msg.pose.position.z=1.5
    # yaw_degrees = setpoints[3]  # North
    yaw_degrees = 90  # North
    yaw = radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)
    print(hx,hy,msg.pose.position.x,msg.pose.position.y)
    msg.pose.orientation = SP.Quaternion(*quaternion)
    return msg

if __name__=='__main__':
    try:     
        follow_point()
    except rospy.ROSInternalException:
        pass
