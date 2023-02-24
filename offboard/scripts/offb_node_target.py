#!/usr/bin/env python3
#read current value of the camera and move in the desired direction
#read the odometry data and use that as the inital position when the mode is shift 


import threading
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
    reached = (dist <= proximity)
    #reached = True

def setGoal(x, y, z, delay=0, wait=True):
    global tx, ty, tz, otx, oty, otz, reached, FirstTarget
    global hx,hy,hz    

    if(FirstTarget):
        otx = x
        oty = y
        otz = z
        print("Current goal: " + str(otx) + " " + str(oty) + " " + str(otz))
        FirstTarget = False
    tx = x
    ty = y
    tz = z
    reached = False
    if wait:
        rate = rospy.Rate(10)
        while not reached and not rospy.is_shutdown():
            rate.sleep()
    print("Goal reached! " + str(otx) + " " + str(oty) + " " + str(otz))
    time.sleep(delay)

def target_callback(aruco):
    global target
    global setpoints
    target=aruco;
    target_x=target.pose.position.x
    target_y=target.pose.position.y
    target_z=target.pose.position.z
    ##The orientation for the target is [roll,pitch,yaw,yaw] in eulers
    target_yaw=target.pose.orientation.z
    setpoints=[target_x,target_y,target_z,target_yaw]


def _CommandThread():
    global otx, oty, otz, tx, ty, tz, reached
    rate = rospy.Rate(10)  # 10hz
    print("waiting for first goal...")
    while(FirstTarget):
        rate.sleep()
    print("publishing goals")
    while not rospy.is_shutdown():
        #reached = True
        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),  # stamp should update
        )
        msg.pose.position.x = otx
        msg.pose.position.y = oty
        msg.pose.position.z = otz
        # print(otx,oty,otz)
        yaw_degrees = 90  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        pub.publish(msg)
        rate.sleep()
    print("command threat quitting...")


rospy.init_node('set_local_setpoint', anonymous=True)
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)

thread.start_new_thread(_CommandThread, ())

pub = SP.get_pub_position_local(queue_size=1)
rospy.Subscriber('target_pose',PoseStamped,target_callback,queue_size=1)
sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, posUpdate, queue_size=1)

while not rospy.is_shutdown():
   #print(hx,hy,hz)
    if (hx!=0.00):
        setGoal(setpoints[0]+hx, setpoints[1]+hy,hz)
        # setGoal(setpoints[0], setpoints[1],setpoints[2])


rospy.spin()
print("Main thred exitting...")
