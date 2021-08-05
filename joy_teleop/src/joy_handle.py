#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Joy_handle():
    def __init__(self):
        rospy.Subscriber('joy',Joy,self.joy_callback)
        self.vel_base_pub = rospy.Publisher('Joy_handle/cmd_base_vel',Twist, queue_size=10)
        self.vel_arm_pub = rospy.Publisher('Joy_handle/cmd_arm_vel',Twist, queue_size=10)
        #base parameters
        self.linear_axis = self.parameter('~linear_axis',4)
        self.angular_axis = self.parameter('~angular_axis',3)
        self.linear_maxVel = self.parameter('~linear_maxVel',0.1)
        self.angular_maxVel = self.parameter('~angular_maxVel',0.5)
        #arm parameters
        self.linear2angular_button = self.parameter('~linear2angular_button',0)
        self.arm_axis_x_pitch = self.parameter('~arm_axis_x_pitch',1)
        self.arm_axis_y_roll = self.parameter('~arm_axis_y_roll',0)
        self.arm_axis_z = self.parameter('~arm_axis_z',7)
        self.arm_axis_yaw = self.parameter('~arm_axis_yaw',6)
        self.arm_xy_maxVel = self.parameter('~arm_xy_maxVel',1)
        self.arm_angular_maxVel = self.parameter('~arm_angular_maxVel',1)
        self.arm_z_vel = self.parameter('~arm_z_vel',1)

    def joy_callback(self,msg):
        #base
        vel_base_msg = Twist()
        vel_base_msg.linear.x = msg.axes[self.linear_axis]*self.linear_maxVel
        vel_base_msg.angular.z = msg.axes[self.angular_axis]*self.angular_maxVel
        self.vel_base_pub.publish(vel_base_msg)
        #arm
        vel_arm_msg = Twist()
        if msg.buttons[self.linear2angular_button]:
            vel_arm_msg.angular.x = msg.axes[self.arm_axis_y_roll]*self.arm_angular_maxVel
            vel_arm_msg.angular.y = msg.axes[self.arm_axis_x_pitch]*self.arm_angular_maxVel
            
        else:
            vel_arm_msg.linear.x = msg.axes[self.arm_axis_x_pitch]*self.arm_xy_maxVel
            vel_arm_msg.linear.y = msg.axes[self.arm_axis_y_roll]*self.arm_xy_maxVel
        
        vel_arm_msg.linear.z = msg.axes[self.arm_axis_z]*self.arm_z_vel
        vel_arm_msg.angular.z = msg.axes[self.arm_axis_yaw]*self.arm_angular_maxVel
        self.vel_arm_pub.publish(vel_arm_msg)

    def parameter(self, name, default):
        if rospy.has_param(name):
            #si existe el parametro, devuelve su valor
            return rospy.get_param(name)
        else:
            #si el parametro no esta definido, devuelve su valor por defecto.
            print("parameter [%s] not defined. Defaulting to %.3f" %(name, default))
            return default


if __name__ == "__main__":
    rospy.init_node("joy_handle")
    Joy_handle()
    rospy.spin()