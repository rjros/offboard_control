#!/usr/bin/env python3
################################################
# Author: Ricardo Rosales
# Date: 2023-02-22
################################################
# Tasks:
#Attitude setpoint test
################################################
import time
import rospy
import numpy as np
import mavros
from mavros import setpoint as SP
from mavros.utils import *
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

#Variable for the PID ROlL
P_roll=5
I_roll=0
D_roll=0
max_roll=5
#Variable for the PID pitch 
P_pitch=5
I_pitch=0
D_pitch=0
max_pitch=5

#Debuggin, testing 1 point
target_pose=np.array([0.0,0.0,0.0])


#should be independent for roll, pitch 
class PID:   
    def __init__(self,P,I,D,max): #define the PID for each axis, and the maximum rate
        self.kp=P
        self.kd=I
        self.ki=D
        self.max=max
        self.targetPos=0
        self.error=0
        #Clear the values of the PID
        self.reset_values()
    
    def reset_values(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.delta_time = 0.1 #dt for the derivative term 
        # Windup Guard
        self.windup_guard = 0.0
        self.output = 0.0

    def update(self, current_pos):
        output_command=0
        error = self.targetPos - current_pos
        delta_error = error - self.last_error  
        self.PTerm = self.kp * error  #
        self.ITerm += error * self.delta_time 
        # print(self.ITerm) 

        # if (self.ITerm > self.windup_guard):  #prevents I term from exceeding 
        #     self.ITerm = self.windup_guard
        # if(self.ITerm < -self.windup_guard):
        #    self.ITerm = -self.windup_guard

        self.DTerm = delta_error / self.delta_time  
        self.last_error = error
        output_command = self.PTerm + (self.ki * self.ITerm) + (self.kd * self.DTerm)
        
        #Bound angles to [-max,max]
        if output_command>=self.max:
            self.output=self.max
        if output_command<=-self.max:
            self.output=-self.max
        else:
            self.output=output_command

        
    def setTargetPosition(self, targetPos):
        self.targetPos = targetPos
        

class Control:
    def __init__(self):
        #Channels RC
        self.mode_select_channel=9#Choose between the attitude controller , the position setpoint controller and position hold 
        self.mode_select=0
        #Commands to be sent in the attitude controller, in degrees
        #limit the roll and pitch 
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.thrust=0.5 #mantain height
        self.quartenion_rot=[0.0,0.0,0.0,0.0]#maybe numpy array problem 

        #State flags 
        self.reached=False
        #Positions and Setpoints from mocap, and targerws        
               
        self.current_pose=PoseStamped()
        self.attitude_setpoints=AttitudeTarget()
        self.attitude_setpoints.header=Header()
        self.attitude_setpoints.body_rate=Vector3()
        self.attitude_setpoints.header.frame_id="base_foorprint"
        self.attitude_setpoints.orientation = Quaternion(*quaternion_from_euler(-0.25,0.15,0))
        self.attitude_setpoints.thrust=0.5
        self.attitude_setpoints.type_mask=7 #ignore body rates
        
        self.pos_x=0.0
        self.pos_y=0.0

    #Use to change between control modes
    def rc_callback(self,msg):
        # 3 Pos switches 1094-1514-1934
        self.RCin=msg.channels
        self.mode_select=self.RCin[self.mode_select_channel-1]         
        if (self.mode_select_channel<=1514):    
            self.mode_position=True
            self.mode_attitude=False 
        else:
            self.mode_position=False
            self.mode_attitude=True 

    def commands(self,roll_cmd,pitch_cmd,stamp):
        self.roll=roll_cmd
        self.pitch=pitch_cmd
        self.attitude_setpoints.header.stamp=stamp
        self.attitude_setpoints.body_rate.x=0
        self.attitude_setpoints.body_rate.y=0
        self.attitude_setpoints.body_rate.z=0



    def send_attitude(self):
        self.quartenion_rot=quaternion_from_euler(self.roll,self.pitch,self.yaw)
        self.attitude_setpoints.orientation.x=self.quartenion_rot[0]
        self.attitude_setpoints.orientation.y=self.quartenion_rot[1]
        self.attitude_setpoints.orientation.z=self.quartenion_rot[2]
        self.attitude_setpoints.orientation.w=self.quartenion_rot[3]
        
        #print(self.attitude_setpoints)
        return self.attitude_setpoints
    
    
    #Position from mocap 
    def pose_callback(self,msg):
        self.current_pose=msg
        self.pos_x=self.current_pose.pose.position.x
        self.pos_y=self.current_pose.pose.position.y


def main():
    global P_roll,I_roll,D_roll,max_roll
    global P_pitch,I_pitch,D_pitch,max_pitch
    global target_pose
    stamp=0

    #Start the node
    rospy.init_node('Offboard_Attitude',anonymous=True)
    control=Control()
    roll_cmd=PID(P_roll,I_roll,D_roll,max_roll)
    pitch_cmd=PID(P_pitch,I_pitch,D_pitch,max_pitch)
    ##Reset values
    roll_cmd.reset_values()
    pitch_cmd.reset_values()
    pub = rospy.Publisher("mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
    rospy.Subscriber('/mavros/rc/in', RCIn, control.rc_callback, queue_size=1)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped,control.pose_callback,queue_size=1)
    #Test for 1 point 
    target_pose[0]=control.pos_x +0.5
    target_pose[1]=control.pos_y #+0.5

    rate = rospy.Rate(100) # 
    while not rospy.is_shutdown():
        # if(control.pos_x!=0.00):
        #Check current position
        roll_cmd.update(control.pos_y)
        pitch_cmd.update(control.pos_x)
        roll_cmd.setTargetPosition(target_pose[1])
        pitch_cmd.setTargetPosition(target_pose[0])
        print(target_pose[1],target_pose[0],roll_cmd.output,pitch_cmd.output)
        #print(control.attitude_setpoints)
        stamp=rospy.Time.now()  # stamp should update
        control.commands(roll_cmd.output,pitch_cmd.output,stamp)
        pub.publish(control.send_attitude())
        rate.sleep()
            
################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#-----------------------------------------------