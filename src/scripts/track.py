#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import numpy as np
import tf.transformations as transformations

class Hoop:
    '''
        Given 3D Pose and radius of a hoop 
        Initial the hoop on rviz - publish visualization markers of a given color (red)
        Track the movement of the drone - subsribe to drone position, and implement collision check / cleared
        Reflect status in rviz in real time
    '''

    def __init__(self, pose, radius, hoop_id):
        '''
            pose: defines the center position & orientation of the hoop (2D plane)
            i.e. given a plane center and normal, find a set of points on that plane with given radius 
        '''
        self.pose = pose
        self.radius = radius
        self.cleared = False #not yet cleared
        self.num_points = 100
        self.hoop_id = hoop_id
        self.marker = self.create_marker()

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"hoop_{self.hoop_id}"
        marker.id = self.hoop_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose = self.pose
        marker.scale.x = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)

        angle_increment = 2 * math.pi / self.num_points
        for i in range(self.num_points + 1):
            angle = i * angle_increment
            y = self.radius * math.cos(angle)
            z = self.radius * math.sin(angle)
            point = Point(self.pose.position.x, y + self.pose.position.y, z + self.pose.position.z)
            marker.points.append(point)
        #make a list of points to make a line strip (math)

        return marker

    def is_collision(self, drone_pos):
        # Transform the drone position to the hoop's coordinate frame
        drone_pos_hoop = self.transform_to_hoop_frame(drone_pos)

        # Calculate the 3-axis distance between the transformed drone position and the hoop's center
        dx = drone_pos_hoop.x
        dy = drone_pos_hoop.y
        dz = drone_pos_hoop.z
        distance = (dx**2 + dy**2)**0.5

        # Check if the drone is in the plane of the hoop and if the distance is less than the hoop's radius
        return (-0.2 <= dz < 0.2) and (distance < self.radius)

    def transform_to_hoop_frame(self, pos):
        """
        Transform a position from the world frame to the hoop's local frame.
        """
        # Create a transformation matrix from the hoop's pose
        t = transformations.translation_matrix([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        q = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        r = transformations.quaternion_matrix(q)
        transform = np.dot(t, r)

        # Apply the transformation to the input position
        pos_hoop = np.dot(transform, [pos.x, pos.y, pos.z, 1])
        return Point(pos_hoop[0], pos_hoop[1], pos_hoop[2])

class HoopManager:
    def __init__(self, yaml_file_path):
        rospy.init_node("hoop_manager_node")
        self.hoops = self.load_hoops_from_yaml(yaml_file_path)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.drone_pose_sub = rospy.Subscriber('/drone/pose', Pose, self.drone_callback)

    def load_hoops_from_yaml(self, file_path):
        #YAML file has position of center in x,y,z and orientation roll, pitch, yaw for every hoop
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)

        hoops = []
        for i, hoop_config in enumerate(config['hoops']):
            pose = Pose()
            pose.position.x = hoop_config['position']['x']
            pose.position.y = hoop_config['position']['y']
            pose.position.z = hoop_config['position']['z']

            roll = math.radians(hoop_config['orientation']['r'])
            pitch = math.radians(hoop_config['orientation']['p'])
            yaw = math.radians(hoop_config['orientation']['y'])
            quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)

            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            radius = hoop_config['radius']
            hoops.append(Hoop(pose, radius, i))

        return hoops

    def publish_markers(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            for hoop in self.hoops:
                if hoop.cleared:
                    hoop.marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # Green
                self.marker_publisher.publish(hoop.marker)
            rate.sleep()

    def drone_callback(self, msg):
        drone_pos = msg.position
        for hoop in self.hoops:
            if not hoop.cleared and hoop.is_collision(drone_pos):
                hoop.cleared = True
                rospy.loginfo(f"Hoop {hoop.hoop_id} cleared!")

if __name__ == '__main__':
    '''
        Use param/arg for name of the track (file stored in configs of package)
        Parse the track YAML file to create hoops / other stuff if added 
        Record & show time until all hoops cleared
    '''
    yaml_file_path = '/home/shaurya/need_for_pry/src/drone_race/config/track1.yaml' #change YAML file path as needed
    hoop_manager = HoopManager(yaml_file_path)
    hoop_manager.publish_markers()
    rospy.spin()