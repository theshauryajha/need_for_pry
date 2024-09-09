#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Display:
    def __init__(self):
        # Initialize the ROS publisher
        self.publisher = rospy.Publisher('/text_marker', Marker, queue_size=10)
        
        # Create a Marker message
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"  # Change this to your frame_id if needed
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "text_marker"
        self.marker.id = 0
        self.marker.type = Marker.TEXT_VIEW_FACING
        self.marker.action = Marker.ADD

        # Set the position where you want the text to appear
        self.marker.pose.position = Point(0, 0, 1) 
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.z = 1.0  # Size of the text
        self.marker.color.a = 1.0  # Alpha channel (transparency)
        self.marker.color.r = 1.0  # Red
        self.marker.color.g = 1.0  # Green
        self.marker.color.b = 1.0  # Blue
        

    def write(self):
        # Update & publish the marker
        self.marker.header.stamp = rospy.Time.now()
        self.marker.text = "Hello " +  str(self.marker.header.stamp.to_sec())
        self.publisher.publish(self.marker)

if __name__ == '__main__':
    rospy.init_node('display_node')
    display = Display()

    rate = rospy.Rate(100)  # Publishing rate in Hz
    while not rospy.is_shutdown():
        display.write()
        rate.sleep()
