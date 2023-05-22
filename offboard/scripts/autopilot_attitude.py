#!/usr/bin/env python3
###########################################################################
# Author: Hannibal Paul
# Date: 2023-03-16
###########################################################################
# UAV attitude control node
# - UAV thrust and yaw from stick during planar control mode. Roll and pitch are 0.
# - UAV roll, pitch, yaw and thrust from stick in normal mode.
###########################################################################
import rospy
import numpy as np
from mavros_msgs.msg import RCIn, AttitudeTarget
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#--------------------------------------------------------------------------

###########################################################################
class AttitudeControl:
    def __init__(self):
        #Subscribers, Publishers
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback, queue_size=1)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback, queue_size=1)
        self.pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        #Variables
        self.current_roll  = 0.0 #deg
        self.current_pitch = 0.0 #deg
        self.current_yaw   = 0.0 #deg

        self.roll_chan   = 0
        self.pitch_chan  = 1
        self.thrust_chan = 2
        self.yaw_chan    = 3
        
        self.uthrust     = 0
        self.uyaw        = 0

        self.THRUST_RATE = 0.1
        self.YAW_RATE = 150 #deg/sec

        self.HOVER_THRUST = 0.70 # simulation only,drone dependent
        
        
        self.stick_center = [1504, 1503, 1501, 1502]

        #stick min and max position for uav control
        #DONT CHANGE: changing these values will chnage the rates
        self.stick_min = -600
        self.stick_max = 600



        #Publisher variables
        self.attitude_control = AttitudeTarget()
        #Set the flags according to requirement. Setting it to 1 will ignore those fields
        #IGNORE_ATTITUDE, IGNORE_THRUST, 0, 0, 0, IGNORE_YAW_RATE, IGNORE_PITCH_RATE, IGNORE_ROLL_RATE
        self.attitude_control.type_mask = self.attitude_control.IGNORE_ATTITUDE#"IGNORE_ATTITUDE"#7#int('0b00000011', 2)
        self.attitude_control.body_rate.x = 0.0
        self.attitude_control.body_rate.y = 0.0
        self.attitude_control.body_rate.z = 0.0
        self.seq = 0
    #--------------------------------------------------------------------------
    def rc_callback(self, msg):
        t_stick = msg.channels[self.thrust_chan]
        y_stick = msg.channels[self.yaw_chan]
        #  ^x (p_stick)
        #  |
        #  --->y (r_stick)
        self.uthrust = t_stick - self.stick_center[self.thrust_chan]
        self.uyaw = y_stick - self.stick_center[self.yaw_chan]
    #--------------------------------------------------------------------------
    def imu_callback(self, msg):
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.current_roll, self.current_pitch, self.current_yaw = np.array(euler_from_quaternion(quaternion)) * 180/np.pi #deg
        #print(self.current_yaw)
    #--------------------------------------------------------------------------
    def send_target(self, roll_rad=0.0, pitch_rad=0.0, yaw_rad=None, yaw_rate=0.0, thrust=0.5):
        #print(thrust)
        if yaw_rad == None:
            yaw_rad = -1.0* np.pi/180 #self.current_yaw
        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        self.attitude_control.header.stamp = rospy.Time.now()
        self.attitude_control.header.seq = self.seq
        # self.attitude_control.orientation.x = quaternion[0]
        # self.attitude_control.orientation.y = quaternion[1]
        # self.attitude_control.orientation.z = quaternion[2]
        # self.attitude_control.orientation.w = quaternion[3]
        
        self.attitude_control.body_rate.x = 0.0#yaw_rate * np.pi/180
        self.attitude_control.body_rate.y = 0.0 # yaw_rate * np.pi/180
        self.attitude_control.body_rate.z = yaw_rate * np.pi/180
        self.attitude_control.thrust = thrust
        self.pub.publish(self.attitude_control)
        #print(self.attitude_control.body_rate.z)
    #--------------------------------------------------------------------------
    def set_attitude(self):
        self.uyaw = 500#50
        y_ctrl = self.YAW_RATE * ((1.0 - -1.0) * (self.uyaw - self.stick_min) / (self.stick_max - self.stick_min) + -1.0)
        self.uthrust = 43
        t_ctrl_temp = self.THRUST_RATE * ((1.0 - -1.0) * (self.uthrust - self.stick_min) / (self.stick_max - self.stick_min) + -1.0)
        t_ctrl = self.HOVER_THRUST + t_ctrl_temp
        print(t_ctrl_temp)
        #self.send_target(0.0, 0.0, None, y_ctrl, t_ctrl)
        self.send_target(0.0, 0.0, None, y_ctrl, t_ctrl)
        self.seq += 1
    #--------------------------------------------------------------------------
        
###########################################################################
def main():
    rospy.init_node('autopilot_attitude', anonymous=True)
    rospy.loginfo('Controlling UAV attitude control from offboard')
    ac = AttitudeControl()
    rate = rospy.Rate(100) #in Hz
    while not rospy.is_shutdown():
        ac.set_attitude()
        rate.sleep()
    rospy.loginfo("Releasing UAV attitude control from offboard")
#--------------------------------------------------------------------------
    
###########################################################################
if __name__ == '__main__':
    #try:
        main()
    #except:
    #    print("Program terminated")
#--------------------------------------------------------------------------