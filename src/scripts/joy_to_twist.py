#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToDrone:
    def __init__(self):
        rospy.init_node('joy_to_drone')
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        
        self.twist = Twist()
        
        # Joystick axis mapping
        self.axis_linear_x = rospy.get_param('~axis_linear_x', 1)  # Left stick up/down
        self.axis_linear_y = rospy.get_param('~axis_linear_y', 0)  # Left stick left/right
        self.axis_linear_z = rospy.get_param('~axis_linear_z', 4)  # Right stick up/down
        self.axis_angular_z = rospy.get_param('~axis_angular_z', 3)  # Right stick left/right
        
        # Sensitivity parameters
        self.scale_linear_xy = rospy.get_param('~scale_linear_xy', 10.0)
        self.scale_linear_z = rospy.get_param('~scale_linear_z', 0.1)
        self.scale_angular_z = rospy.get_param('~scale_angular_z', 3.0)
        
        # Exponential factor for fine control
        self.expo_factor = rospy.get_param('~expo_factor', 0.5)

        # Deadzone
        self.deadzone = rospy.get_param('~deadzone', 0.05)

    def apply_expo(self, value, expo):
        return value * (abs(value) ** expo)

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        return value

    def joy_callback(self, joy_msg):
        # Left stick for x and y movement
        x = self.apply_deadzone(joy_msg.axes[self.axis_linear_x], self.deadzone)
        y = self.apply_deadzone(joy_msg.axes[self.axis_linear_y], self.deadzone)
        
        self.twist.linear.x = self.scale_linear_xy * self.apply_expo(x, self.expo_factor)
        self.twist.linear.y = self.scale_linear_xy * self.apply_expo(y, self.expo_factor)
        
        # Right stick up/down for z movement
        z = self.apply_deadzone(joy_msg.axes[self.axis_linear_z], self.deadzone)
        self.twist.linear.z = self.scale_linear_z * self.apply_expo(z, self.expo_factor)
        
        # Right stick left/right for yaw control
        yaw = self.apply_deadzone(joy_msg.axes[self.axis_angular_z], self.deadzone)
        self.twist.angular.z = self.scale_angular_z * self.apply_expo(yaw, self.expo_factor)
        
        # We're not using pitch and roll for angular velocity, so set them to 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        
        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        joy_to_drone = JoyToDrone()
        joy_to_drone.run()
    except rospy.ROSInterruptException:
        pass