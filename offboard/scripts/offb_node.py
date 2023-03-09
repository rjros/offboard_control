#!/usr/bin/env python3
#read current value of the camera and move in the desired direction
#read the odometry data and use that as the inital position when the mode is shift 


import threading
import time
from math import *

import rospy
import _thread as thread
from tf.transformations import quaternion_from_euler

import mavros
from mavros import setpoint as SP
from mavros.utils import *
import numpy as np

####Value received by the T camera 
from nav_msgs.msg import Odometry


# Params
proximity = 0.1 # metres

# Variable
FirstTarget = True
otx, oty, otz, oth = (0.0,0.0,0.0,0.0)
tx, ty, tz, th = (0.0,0.0,0.0,0.0)
cx, cy, cz, ch = (0.0,0.0,0.0,0.0)
hx, hy, hz=(0.0,0.0,0.0)

reached = True
moving = False
state = False

def posUpdate(msg):
    global reached
    global state
    global hx,hy,hz
    global proximity
    global otx, oty, otz

    if (state==False):
        hx=round(msg.pose.position.x,3)
        hy=round(msg.pose.position.y,3)
        hz=round(msg.pose.position.z,3)
        state=True
    mx = msg.pose.position.x
    my = msg.pose.position.y
    mz = msg.pose.position.z
    dist = sqrt(pow(mx-otx, 2) + pow(my-oty, 2) + pow(mz-otz, 2))
    reached = (dist <= proximity)
    #reached = True

def setGoal(x, y, z, delay=0, wait=True):
    global tx, ty, tz, otx, oty, otz, reached, FirstTarget
    global hx,hy,hz
    print (round(hx,3),round(hy,3),round(hz,3))
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
        if(otx != tx or oty != ty or otz != tz): # Set goal has changed
            if(reached): # If the drone has reached the last position
                otx = tx
                oty = ty
                otz = tz
                print("Current goal: " + str(otx) + " " + str(oty) + " " + str(otz))
        msg.pose.position.x = otx
        msg.pose.position.y = oty
        msg.pose.position.z = otz
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
sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, posUpdate, queue_size=1)

#print("Publishing first goal in 3 seconds...")
#time.sleep(3)
#x is left
#y is up, it matches with the mocap

setpoints = np.array([
    [0.0, 0.0, 0.5, 0.5],
    [0.8, 0.0, 0.5, 0.5],
    [0.8, 0.8, 0.5, 0.5],
    [0.0, 0.8, 0.5, 0.5]
])
i = 0
##Add inital home position when the program started 
while not rospy.is_shutdown():
    #print(hx,hy,hz)
    if (hx!=0.00):
        setGoal(setpoints[i][0] + hx, setpoints[i][1]+ hy ,setpoints[i][2], setpoints[i][3])
        i = i+1
        if(i >= setpoints.size/4):
            i = 0

rospy.spin()
print("Main thred exitting...")
