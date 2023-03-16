#!/usr/bin/env python3
################################################
# Tasks:
#-Sends Attitude commands test
#-Allows the use of the rc during offboard mode
################################################
###########################################################################
__author__ = "Ricardo Rosales, Hannibal Paul, and Borwonpob Sumetheeprasit"
__copyright__ = "-"
__credits__ = ["-"]
__license__ = "-"
__version__ = "1.0.1"
__maintainer__ = "- "
__email__ = " -"
__status__ = "-"
###########################################################################
import time
import rospy
import numpy as np
from math import pi
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
P_roll=0.0
I_roll=0.0
D_roll=0
max_roll=2
#Variable for the PID pitch 
P_pitch=0.0
I_pitch=0.0
D_pitch=0
max_pitch=2

#Variable for the PID thrust 
P_thrust=0.0
I_thrust=0.0
D_thrust=0.0
max_thrust=0.1
min_thrust=-0.1 #system just needs to lower from the hover state 

#Max angle in degrees for stick control roll,pitch and yaw 
max_angle=35

#Debugging, testing 1 point
target_pose=np.array([0.0,0.0,0.0])


#should be independent for roll, pitch 
class PID:   
    def __init__(self,P,I,D,max,min): #define the PID for each axis, and the maximum rate
        self.kp=P
        self.ki=I
        self.kd=D
        self.max=max
        self.min=min
        self.targetPos=0.0
        self.error=0.0
        #Clear the values of the PID
        self.reset_values()
    
    def reset_values(self):
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
            # print ('Max',self.max,output_command)
            output_command=self.max
        if output_command<=self.min:
            # print ('Min',self.max,output_command)
            output_command=self.min
        
        self.output=output_command

        
    def setTargetPosition(self, targetPos):
        self.targetPos = targetPos
        

class Control:
    def __init__(self):
        #Channels RC
        self.roll_select_channel=1
        self.pitch_select_channel=2#reverse channel in code
        self.altitude_select_channel=3
        self.yaw_select_channel=4
        self.mode_select_channel=8
        #Use the RC sticks to control the drone for debugging or semi-autonomous work
        self.mode_select=0   #Choose between the attitude controller ,the position setpoint controller and position hold (not implemented)
        self.roll_channel=0  #1082-1921
        self.pitch_channel=0 #1919-1079
        self.yaw_channel=0  #1082-1921
        self.altitude_channel=0 #1029-1934 
        #Commands to be sent in the attitude controller, in degrees
        #limit the roll and pitch 
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.thrust=0.0 #maintain height
        self.hover=0.55 #for simulation confirm with actual drone 
        self.angle=10 #maximum tilt angle
        self.quatenion_rot=[0.0,0.0,0.0,0.0]#maybe numpy array problem 

        #State flags 
        self.reached=False
        #Positions and Setpoints from mocap, and targerws    
        #Mavros State
        self.state=State()
        self.offboard_mode=False

        #Debugging Topics
        self.plot=Vector3()    
               
        self.current_pose=PoseStamped()
        self.attitude_setpoints=AttitudeTarget()
        self.attitude_setpoints.header=Header()
        self.attitude_setpoints.body_rate=Vector3()
        self.attitude_setpoints.header.frame_id="base_foorprint"
        self.attitude_setpoints.orientation = Quaternion(*quaternion_from_euler(0,0,0))
        self.attitude_setpoints.thrust=self.hover
        self.attitude_setpoints.type_mask=7 #ignore body rates
        self.pos_x=0.0
        self.pos_y=0.0
        self.pos_z=0.0
    
    #use for setting additional control parameters
    def parameters(self,max_angle):
        self.angle=max_angle
    
    def map(self,OldValue, OldMin, OldMax, NewMin, NewMax):
        return (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    
    #Use to change between control modes
    def rc_callback(self,msg):
        # 3 Pos switches 1094-1514-1934
        self.RCin=msg.channels
        self.mode_select=self.RCin[self.mode_select_channel-1]
        self.roll_channel=self.map(self.RCin[self.roll_select_channel-1],1082,1921,-self.angle,self.angle)
        self.pitch_channel=self.map(self.RCin[self.pitch_select_channel-1],1079,1919,-self.angle,self.angle)
        self.yaw_channel=self.map(self.RCin[self.yaw_select_channel-1],1082,1921,-self.angle,self.angle)
        ##small values are enough to change altitude##
        self.altitude_channel=self.map(self.RCin[self.altitude_select_channel-1],1082,1921,-0.1,0.1)
        # print(self.roll_channel,self.pitch_channel,self.yaw_channel,self.altitude_channel)

        self.mode_select=self.RCin[self.mode_select_channel-1]         
        if (self.mode_select_channel<=1514):    
            self.mode_position=True
            self.mode_attitude=False 
        else:
            self.mode_position=False
            self.mode_attitude=True 

    def commands(self,roll_cmd,pitch_cmd,yaw_cmd,thrust_cmd,stamp):
        
        #update stamp
        self.attitude_setpoints.header.stamp=stamp
        
        if (self.mode_select < 1400):
            ##Using values from the stick##
            self.roll=self.roll_channel*pi/180
            self.pitch=-self.pitch_channel*pi/180
            self.yaw= 90*pi/180 #+self.yaw_channel*pi/180
            #print(self.roll_channel,self.pitch_channel,self.yaw_channel)
        else:  
        ##Fixed value or value from PID##
            self.roll=0 #roll_cmd
            self.pitch=0 #pitch_cmd
            self.yaw= 90*pi/180  #yaw_cmd
       
        # print(self.roll,self.pitch,self.yaw)

        ##Fixed##
        self.attitude_setpoints.body_rate.x=0
        self.attitude_setpoints.body_rate.y=0
        self.attitude_setpoints.body_rate.z=0
        ##Thrust command [0,1] test + hover state ~0.67##
        # self.attitude_setpoints.thrust=self.hover + thrust_cmd
        self.attitude_setpoints.thrust=self.hover + self.altitude_channel


    def send_attitude(self):
        self.quaternion_rot=quaternion_from_euler(self.roll,self.pitch,self.yaw)
        # print(self.quaternion_rot)
        self.attitude_setpoints.orientation.x=self.quaternion_rot[0]
        self.attitude_setpoints.orientation.y=self.quaternion_rot[1]
        self.attitude_setpoints.orientation.z=self.quaternion_rot[2]
        self.attitude_setpoints.orientation.w=self.quaternion_rot[3]

        
        #print(self.attitude_setpoints)
        return self.attitude_setpoints
    
    
    #Position from mocap 
    def pose_callback(self,msg):
        self.current_pose=msg
        self.pos_x=self.current_pose.pose.position.x
        self.pos_y=self.current_pose.pose.position.y
        self.pos_z=self.current_pose.pose.position.z
    
    #UAV State[armed,connected, guided,manual_input,mode]
    def state_callback(self,msg):
        self.state=msg
        if (self.state.mode=='OFFBOARD'):
            self.offboard_mode=True
        else:
            self.offboard_mode=False

    def plot_data(self,p1,p2,p3):
        self.plot.x=p1
        self.plot.y=p2
        self.plot.z=p3
        return self.plot
def main():
    global P_roll,I_roll,D_roll,max_roll
    global P_pitch,I_pitch,D_pitch,max_pitch
    global P_thrust,I_thrust,D_thrust,max_thrust
    global target_pose,max_angle
    stamp=0
    #For checking fixed orientations only. Comment out  when not needed
    roll_test=0.0
    pitch_test=0.0
    yaw_test=90.0 #value in degrees, depends positioning method [in mocap facing towards the front] 
    ############################################################################################
    #Start the node
    rospy.init_node('Offboard_Attitude',anonymous=True)
    control=Control()
    control.parameters(max_angle)
    ##PID Setup##
    roll_cmd=PID(P_roll,I_roll,D_roll,max_roll,-max_roll)
    pitch_cmd=PID(P_pitch,I_pitch,D_pitch,max_pitch,-max_pitch)
    thrust_cmd=PID(P_thrust,I_thrust,D_thrust,max_thrust,min_thrust)
    ##Reset values
    roll_cmd.reset_values()
    pitch_cmd.reset_values()
    thrust_cmd.reset_values()

    pub = rospy.Publisher("mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
    rospy.Subscriber('/mavros/rc/in', RCIn, control.rc_callback, queue_size=1)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped,control.pose_callback,queue_size=1)
    rospy.Subscriber('mavros/state',State,control.state_callback,queue_size=1)

    #Use for debugging 
    #pub_plot=rospy.Publisher("plot_data",Vector3,queue_size=1)
    
    #Test for 1 point 
    while (control.pos_z==0.0):
        print('Waiting for position estimate')
        pass
    print('Position received')
    target_pose[0]=control.pos_x #+0.5
    target_pose[1]=control.pos_y #+0.5
    target_pose[2]=control.pos_z #maintain height depends motor

    rate = rospy.Rate(100) # 
    while not rospy.is_shutdown():
        if (control.offboard_mode):   
            # if(control.pos_x!=0.00):
            #Send desired position
            roll_cmd.setTargetPosition(target_pose[1])
            pitch_cmd.setTargetPosition(target_pose[0])
            thrust_cmd.setTargetPosition(target_pose[2])
            #Check current position
            roll_cmd.update(control.pos_y)
            pitch_cmd.update(control.pos_x)
            thrust_cmd.update(control.pos_z)
            # print(target_pose[1],target_pose[0],roll_cmd.output,pitch_cmd.output)
            #print(control.attitude_setpoints)
             #send attitude commands 
            # print(roll_cmd.output,pitch_cmd.output,thrust_cmd.output)
            # print('Current Altitude',control.pos_z,'Altitude target',target_pose[2],'P_error',thrust_cmd.PTerm,
            #       'Command',thrust_cmd.output,'thrust',control.attitude_setpoints.thrust)
            # print('Pterm',thrust_cmd.PTerm,'Item',thrust_cmd.ITerm,'Dterm',thrust_cmd.DTerm,'CMD',control.attitude_setpoints.thrust)

        else:
            roll_cmd.reset_values()
            pitch_cmd.reset_values()
            thrust_cmd.reset_values()
        stamp=rospy.Time.now()  # stamp should update

        #####################################################################################
        #####################3Full Controller################################################
        # control.commands(roll_cmd.output,pitch_cmd.output,yaw_cmd,thrust_cmd.output,stamp)
        #####################################################################################
        
        #For testing a fixed height and no rotation
        control.commands(roll_test,pitch_test,yaw_test,thrust_cmd.output,stamp)
        pub.publish(control.send_attitude())
        #pub_plot.publish(control.plot_data(target_pose[2],control.pos_z,control.attitude_setpoints.thrust))
        #print('Pterm',thrust_cmd.PTerm,'P output',thrust_cmd.output,'CMD',control.attitude_setpoints.thrust)
        rate.sleep()
            
################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#-----------------------------------------------