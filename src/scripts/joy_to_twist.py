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
        
        # Adjust these values based on your joystick
        self.axis_linear_x = 1  # Left stick up/down
        self.axis_linear_y = 0  # Left stick left/right
        self.axis_linear_z = 3  # Right stick up/down
        self.axis_angular_z = 2  # Right stick left/right
        
        self.scale_linear = 1.0
        self.scale_angular = 1.0

    def joy_callback(self, joy_msg):
        self.twist.linear.x = self.scale_linear * joy_msg.axes[self.axis_linear_x]
        self.twist.linear.y = self.scale_linear * joy_msg.axes[self.axis_linear_y]
        self.twist.linear.z = self.scale_linear * joy_msg.axes[self.axis_linear_z]
        self.twist.angular.z = self.scale_angular * joy_msg.axes[self.axis_angular_z]
        
        self.pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        joy_to_drone = JoyToDrone()
        joy_to_drone.run()
    except rospy.ROSInterruptException:
        pass