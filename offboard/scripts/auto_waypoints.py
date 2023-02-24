#!/usr/bin/env python3
################################################
# Author: Hannibal Paul
# Date: 2023-01-27
################################################
# Waypoint publisher for autonomous flight
# -Reads current position from tracking camera(or MOCAP) and move in the desired direction and yaw orientatioin
# -The inital position of MOCAP is used and set as 0 refrence in X,Y. 
###########################################################################
import time
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import mavros
from mavros_msgs.msg import RCIn 
from mavros import setpoint as SP
from mavros.utils import *
import numpy as np
from nav_msgs.msg import Odometry
from servo_control.msg import ArmsWrite
# test with geometry msgs
#--------------------------------------------------------------------------

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


################################################
class CONTROL_TASK:
    def __init__(self):
        #-----------------------------------------------
        #RC channel
        self.tilt_activate_channel = 8 # use 2/3 position switch
        self.angle_change_channel = 9 # use multi position switch
        self.speed_control_channel = 10 # use multi position switch
        self.stay_position_channel = 12 

        # Accuracy for target positioning
        self.PROXIMITY = 0.15 # in metres
        self.YAW_TOLERANCE = 1 # in degrees

        # Various signals from RC
        self.activate_signal = 0
        self.angle_signal = 0
        self.speed_signal = 0
        self.stay_position_signal = 0

        # Control flag
        self.START_OFFSET = True
        self.angle_control_active = False
        self.angle_change_immidiate = False
        self.INITIAL_POSE_RECEIVED = False
        self.TARGET_REACHED = False
        self.MOVING = False
        #-----------------------------------------------
        #Variables to store the subscribed RC data
        self.rc_in = []
        #-----------------------------------------------
        # Publish type msg
        self.data_pub = ArmsWrite()
        
        # Variables
        self.initial_position = np.array((0.0, 0.0, 0.0))
        self.current_position = np.array((0.0, 0.0, 0.0))
        self.prev_target = np.array((0.0, 0.0, 0.0, 0.0))


        self.angle_degree = 0
        self.angle_dir = 1
    #-----------------------------------------------
    #-----------------------------------------------
    def map_function(self, value_to_map, current_min, current_max, new_min, new_max):
        return (new_max - new_min)*(value_to_map - current_min) / (current_max - current_min) + new_min
    #-----------------------------------------------
    #-----------------------------------------------
    def rc_callback(self, msg):
        self.rc_in = msg.channels # 1094 to 1934
        #print(self.rc_in)
        self.activate_signal= self.rc_in[self.tilt_activate_channel-1]
        self.angle_signal = self.rc_in[self.angle_change_channel-1]
        self.speed_signal = self.rc_in[self.speed_control_channel-1]
        self.stay_position_signal = self.rc_in[self.stay_position_channel-1]
    #-----------------------------------------------
    #-----------------------------------------------
    def motor_control(self, pub):
        self.angle_dir = -1
        if self.activate_signal > 1900:
            self.angle_dir = 1
        elif self.activate_signal > 1500:
            self.angle_dir = 0
            


        if self.angle_signal > 1900:
            self.angle_change_immidiate = True
        else:
            self.angle_change_immidiate = False

        if self.stay_position_signal > 1900:
            self.angle_degree = 90 * self.angle_dir
        elif self.stay_position_signal > 1500:
            self.angle_degree = 45 * self.angle_dir
        else:
            self.angle_degree = 0


        speed_control = int(self.map_function(self.speed_signal, 1094, 1934, 1024, 1)) #1024

        self.data_pub.mode = False
        self.data_pub.torque = True
        self.data_pub.speed = speed_control

        
        #if self.angle_control_active:
        self.data_pub.angles_abs = [self.angle_degree, -self.angle_degree]
        #else:
        #    self.data_pub.angles_abs = [0, 0]

        #print(self.data_pub.angles_abs)
        
        pub.publish(self.data_pub)
    #-----------------------------------------------
    #-----------------------------------------------
    def pos_update(self, msg):
        # Save initial position for refrence
        if (self.INITIAL_POSE_RECEIVED == False):
            self.initial_position[0] = round(msg.pose.position.x, 3)
            self.initial_position[1] = round(msg.pose.position.y, 3)
            self.initial_position[2] = round(msg.pose.position.z, 3)
            self.INITIAL_POSE_RECEIVED = True

        # Position
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z
        # Orientation
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        current_yaw = yaw * 180/3.14
        
        # Check Eucledian distance
        dist_error = np.linalg.norm(self.prev_target[:3] - self.current_position)
        ang_error = abs(self.prev_target[3] - current_yaw) 

        if (dist_error <= self.PROXIMITY) and (ang_error <= self.YAW_TOLERANCE):
            self.TARGET_REACHED = True
            self.MOVING = False
            rospy.loginfo("Target reached: %f %f %f %f", self.prev_target[0], self.prev_target[1], self.prev_target[2], self.prev_target[3])
        else:
            self.MOVING = True
            #rospy.loginfo("Moving to target!")
        #rospy.loginfo("Current position: [%f %f %f]", current_position[0] - initial_position[0], current_position[1] - initial_position[1], current_position[2])
    #-----------------------------------------------
    #-----------------------------------------------
    def set_goal(self, target, pub):
        target_x, target_y, target_z, target_yaw, delay = target
        if self.START_OFFSET:
            target_x += self.initial_position[0]
            target_y += self.initial_position[1]
        # print(target_x, target_y, target_z, target_yaw, delay)
        
        msg = SP.PoseStamped(
            header=SP.Header(frame_id="base_footprint", stamp=rospy.Time.now()),)
        # Position        
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        # Orientation
        yaw = target_yaw * 3.14/180 #in rad #90- North
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        pub.publish(msg)

        if self.TARGET_REACHED: # Change to next target
            self.TARGET_REACHED = False
            if((self.prev_target[0] != target_x) or (self.prev_target[1] != target_y) or (self.prev_target[2] != target_z)):
                self.prev_target[0] = target_x
                self.prev_target[1] = target_y
                self.prev_target[2] = target_z
                self.prev_target[3] = target_yaw
            time.sleep(delay)
    #-----------------------------------------------

################################################
def task_routine():
    global INITIAL_POSE_RECEIVED, TARGET_REACHED, MOVING
    global initial_position, current_position, prev_target, PROXIMITY, YAW_TOLERANCE

    #-----------------------------------------------
    #Start the ROS node
    rospy.init_node('Task_conrol_setpoint', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    rospy.loginfo("Waypoint script active!")
    control_task = CONTROL_TASK()
    #-----------------------------------------------
    pub_pose = SP.get_pub_position_local(queue_size=1)
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, control_task.pos_update, queue_size=1)
    pub_servo = rospy.Publisher('/dxl_servo/angle_control', ArmsWrite, queue_size=1)
    rospy.Subscriber('/mavros/rc/in', RCIn, control_task.rc_callback, queue_size=1)
    #-----------------------------------------------
    #x is right, y is front, z is up
    #format = [x, y, z, yaw, delay in s]
    setpoints = np.array([
        [0.0, 0.0, 1.0, 90, 1.0],
        [0.8, 0.0, 1.0, 90, 1.0],
        [0.8, 0.8, 1.0, 90, 1.0],
        [0.0, 0.8, 1.0, 90, 1.0]
        ])

    angles = np.array([
        [0],
        [-45],
        [-90],
        [-45],
        [0],
        [45],
        [90],
        [45],
        [0]
        ])

    rate = rospy.Rate(10)
    setpoint_index = 0
    angle_index = 0
    #once = 
    #control_task.motor_control(angles[0], pub_servo)
    # Inital position is the home position - when the program starts
    while not rospy.is_shutdown():
        #Wait till the current MOCAP data is received
        #control_task.INITIAL_POSE_RECEIVED = True


        control_task.motor_control(pub_servo)

        '''
        if control_task.INITIAL_POSE_RECEIVED:
            
            #print(initial_position)
            #rospy.loginfo("Sending waypoint %d: [%f %f %f]", setpoint_index, setpoints[setpoint_index][0], setpoints[setpoint_index][1], setpoints[setpoint_index][2])
            #rospy.loginfo("Sending waypoint %d", setpoint_index)

            if not(control_task.angle_control_active):
                angle_index = 0

            control_task.motor_control(angles[angle_index], pub_servo)
            
            if control_task.position_move:
                control_task.set_goal(setpoints[setpoint_index], pub_pose)
            else:
                control_task.set_goal(setpoints[0], pub_pose)
                time.sleep(5)
                


            # control_task.TARGET_REACHED = True

            if control_task.TARGET_REACHED:
                setpoint_index += 1
                if control_task.angle_change_immidiate:
                    angle_index += 1
                    if(angle_index >= angles.shape[0]):
                        rospy.loginfo("Waypoint task finished!")
                        break
            if(setpoint_index >= setpoints.shape[0]):
                setpoint_index = 0
                if not(control_task.angle_change_immidiate):
                    angle_index += 1
                    if(angle_index >= angles.shape[0]):
                        rospy.loginfo("Waypoint task finished!")
                        break
        '''
    #-----------------------------------------------
    rospy.spin()
    rospy.loginfo("Waypoint script exited!")
#-----------------------------------------------
    
################################################
if __name__ == '__main__':
    try:
        task_routine()
    except rospy.ROSInterruptException:
        pass
#-----------------------------------------------
