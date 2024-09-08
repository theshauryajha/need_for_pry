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
        self.axis_r2 = rospy.get_param('~axis_r2', 5)  # R2 trigger
        self.axis_l2 = rospy.get_param('~axis_l2', 2)  # L2 trigger
        
        # Sensitivity parameters
        self.scale_linear_xy = rospy.get_param('~scale_linear_xy', 1.25)
        self.scale_linear_z = rospy.get_param('~scale_linear_z', 0.25)
        
        # Exponential factor for fine control
        self.expo_factor = rospy.get_param('~expo_factor', 0.5)

        # Deadzone
        self.deadzone = rospy.get_param('~deadzone', 0.5)

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
        
        # R2 and L2 for up and down
        # Note: Triggers usually output -1 when not pressed and 1 when fully pressed
        r2 = (-joy_msg.axes[self.axis_r2] + 1) / 2  # Convert to 0-1 range
        l2 = (-joy_msg.axes[self.axis_l2] + 1) / 2  # Convert to 0-1 range
        
        z_velocity = (r2 - l2) * self.scale_linear_z
        self.twist.linear.z = self.apply_expo(z_velocity, self.expo_factor)
        
        # We're not using angular velocity, so set it to 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        
        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        joy_to_drone = JoyToDrone()
        joy_to_drone.run()
    except rospy.ROSInterruptException:
        pass