#!/usr/bin/env python3
# Waypoint publisher for autonomous flight
# -Reads current position from tracking camera(or MOCAP) and move in the desired direction and yaw orientatioin
# -The inital position of MOCAP is used and set as 0 refrence in X,Y. 
###########################################################################
__author__ = "Hannibal Paul, Ricardo Rosales and Borwonpob Sumetheeprasit"
__copyright__ = "-"
__credits__ = ["-"]
__license__ = "-"
__version__ = "1.0.1"
__maintainer__ = "- "
__email__ = " -"
__status__ = "-"
###########################################################################
import threading
import time
#from math import *
import rospy
#import _thread as thread
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import mavros
from mavros import setpoint as SP
from mavros.utils import *
import numpy as np
from nav_msgs.msg import Odometry
#--------------------------------------------------------------------------
# Accuracy for target positioning
PROXIMITY = 0.15 # in metres
YAW_TOLERANCE = 1 # in degrees

# Variables
initial_position = np.array((0.0, 0.0, 0.0))
current_position = np.array((0.0, 0.0, 0.0))
prev_target = np.array((0.0, 0.0, 0.0, 0.0))

# Flags
INITIAL_POSE_RECEIVED = False
TARGET_REACHED = True
MOVING = False
#--------------------------------------------------------------------------
def posUpdate(msg):
    global INITIAL_POSE_RECEIVED, TARGET_REACHED, MOVING
    global initial_position, current_position, prev_target, PROXIMITY, YAW_TOLERANCE

    # Save initial position for refrence
    if (INITIAL_POSE_RECEIVED == False):
        initial_position[0] = round(msg.pose.position.x, 3)
        initial_position[1] = round(msg.pose.position.y, 3)
        initial_position[2] = round(msg.pose.position.z, 3)
        INITIAL_POSE_RECEIVED = True

    # Position
    current_position[0] = msg.pose.position.x
    current_position[1] = msg.pose.position.y
    current_position[2] = msg.pose.position.z
    # Orientation
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_yaw = yaw * 180/3.14
    
    # Check Eucledian distance
    dist_error = np.linalg.norm(prev_target[:3] - current_position)
    ang_error = abs(prev_target[3] - current_yaw) 

    if (dist_error <= PROXIMITY) and (ang_error <= YAW_TOLERANCE):
        TARGET_REACHED = True
        MOVING = False
        rospy.loginfo("Target reached: %f %f %f %f", prev_target[0], prev_target[1], prev_target[2], prev_target[3])
    else:
        MOVING = True
        #rospy.loginfo("Moving to target!")
    #rospy.loginfo("Current position: [%f %f %f]", current_position[0] - initial_position[0], current_position[1] - initial_position[1], current_position[2])
#--------------------------------------------------------------------------
def setGoal(target_x, target_y, target_z, target_yaw, delay, pub):
    global TARGET_REACHED
    global current_position, prev_target

    msg = SP.PoseStamped(
        header=SP.Header(frame_id="base_footprint", stamp=rospy.Time.now()),)
    # Position
    msg.pose.position.x = target_x
    msg.pose.position.y = target_y
    msg.pose.position.z = target_z
    # Orientation
    yaw_degrees = target_yaw #90  # North
    yaw = yaw_degrees * 3.14/180
    quaternion = quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation = SP.Quaternion(*quaternion)
    pub.publish(msg)

    if TARGET_REACHED: # Change to next target
        TARGET_REACHED = False
        if((prev_target[0] != target_x) or (prev_target[1] != target_y) or (prev_target[2] != target_z)):
            prev_target[0] = target_x
            prev_target[1] = target_y
            prev_target[2] = target_z
            prev_target[3] = target_yaw
        time.sleep(delay)
#--------------------------------------------------------------------------
def main():
    rospy.init_node('set_local_setpoint', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)
    rospy.loginfo("Waypoint script started!")

    pub = SP.get_pub_position_local(queue_size=1)
    sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, posUpdate, queue_size=1)
    
    #x is right, y is front, z is up
    #format = [x, y, z, yaw, delay in s]
    setpoints = np.array([
        [0.0, 0.0, 1.0, 90, 1.0],
        [0.8, 0.0, 1.0, 90, 1.0],
        [0.8, 0.8, 1.0, 90, 1.0],
        [0.0, 0.8, 1.0, 90, 1.0]
        ])

    setpoint_index = 0
    # Inital position is the home position - when the program starts
    while not rospy.is_shutdown():
        #Wait till the current MOCAP data is received
        if INITIAL_POSE_RECEIVED:
            #print(initial_position)
            #rospy.loginfo("Sending waypoint %d: [%f %f %f]", setpoint_index, setpoints[setpoint_index][0], setpoints[setpoint_index][1], setpoints[setpoint_index][2])
            setGoal(setpoints[setpoint_index][0] + initial_position[0], 
            setpoints[setpoint_index][1] + initial_position[1], 
            setpoints[setpoint_index][2], 
            setpoints[setpoint_index][3],
            setpoints[setpoint_index][4], pub)
            
            #while not TARGET_REACHED:
            #    rate.sleep()
            if TARGET_REACHED:
                setpoint_index += 1
            if(setpoint_index >= setpoints.shape[0]):
                setpoint_index = 0
                time.sleep(5)

    rospy.spin()
    rospy.loginfo("Waypoint script exited!")
#--------------------------------------------------------------------------
if __name__ == "__main__":
   main()
#--------------------------------------------------------------------------
