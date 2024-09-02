#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import tf.transformations as transformations  # Use to convert Euler angles to quaternions

class Hoop:
    '''
        Given 3D Pose and radius of a hoop 
        Initialize the hoop on rviz - publish visualization markers of a given color (red)
        Track the movement of the drone - subscribe to drone position, and implement collision check / cleared
        Reflect status in rviz in real time
    '''
    def __init__(self, pose, radius):
        '''
            pose: defines the center position & orientation of the hoop (2D plane)
            i.e. given a plane center and normal, find a set of points on that plane with given radius 
        '''
        rospy.init_node("hoop_node")  # Initialize the ROS node

        self.pose = pose
        self.radius = radius
        self.cleared = False
        self.num_points = 100
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.drone_pose_sub = rospy.Subscriber('/drone/pose', Pose, self.drone_callback)

        self.marker = self.create_marker()

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "hoop"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose = self.pose

        # Set the scale: thickness of the line
        marker.scale.x = 0.1  # Thickness of the hoop's line

        # Set the color: red with 50% transparency
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # Red color

        # Generate the circle points in the YZ plane with a constant X coordinate
        angle_increment = 2 * math.pi / self.num_points
        for i in range(self.num_points + 1):  # +1 to close the loop
            angle = i * angle_increment
            y = self.radius * math.cos(angle)
            z = self.radius * math.sin(angle)
            point = Point()
            point.x = self.pose.position.x  # Keep the x position constant
            point.y = y + self.pose.position.y
            point.z = z + self.pose.position.z
            marker.points.append(point)

        return marker
    
    def publish_marker(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.cleared:
                rospy.loginfo("Marker cleared, changing color to green...")
                self.marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Change color to green
            else:
                rospy.loginfo("Publishing marker...")
            self.marker_publisher.publish(self.marker)
            rate.sleep()

    def drone_callback(self, msg):
        drone_pos = msg.position
        if self.is_collision(drone_pos):
            self.cleared = True

    def is_collision(self, drone_pos):
        # Assuming self.pose.position is the center of the hoop
        dy = drone_pos.y - self.pose.position.y
        dz = drone_pos.z - self.pose.position.z
        distance = (dy**2 + dz**2)**0.5
        return distance < self.radius

def load_hoop_from_yaml(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)

    hoop_config = config['hoop1']
    
    # Set up pose
    pose = Pose()
    pose.position.x = hoop_config['position']['x']
    pose.position.y = hoop_config['position']['y']
    pose.position.z = hoop_config['position']['z']

    # Convert Euler angles to quaternion
    roll = math.radians(hoop_config['orientation']['r'])
    pitch = math.radians(hoop_config['orientation']['p'])
    yaw = math.radians(hoop_config['orientation']['y'])
    quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)

    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    # Get radius
    radius = hoop_config['radius']

    return pose, radius

if __name__ == '__main__':
    '''
        Load track information from a YAML file
        Initialize hoops from the YAML data
    '''
    yaml_file_path = '/home/shaurya/need_for_pry/src/drone_race/config/track1.yaml'  # Replace with actual path to your YAML file
    hoop_pose, hoop_radius = load_hoop_from_yaml(yaml_file_path)

    hoop = Hoop(hoop_pose, hoop_radius)
    hoop.publish_marker()

    rospy.spin()
