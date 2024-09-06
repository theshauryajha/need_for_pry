#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import numpy as np
import tf.transformations as transformations
from scipy.spatial.transform import Rotation as R

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
        self.cleared = False # Not yet cleared
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
        # Make a list of points to make a line strip (math)

        return marker

    def is_collision(self, drone_pos):
        # Transform the drone position to the hoop's coordinate frame
        drone_pos_hoop = self.transform_to_hoop_frame(drone_pos)

        # Calculate the 2D distance between the drone and the center of the hoop (x, y plane)
        dx = drone_pos_hoop[0]
        dy = drone_pos_hoop[1]
        dz = drone_pos_hoop[2]
        distance = (dx**2 + dy**2 + dz**2)**0.5

        # Check if the drone is within the hoop's radius and near the hoop's plane (z ~ 0)
        return (-0.2 <= dz <= 0.2) and (distance < self.radius)


    def transform_to_hoop_frame(self, pos):
        # Extract the quaternion from the pose
        quaternion = [
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ]

        # Convert the quaternion to a 3x3 rotation matrix
        rotation = R.from_quat(quaternion).as_dcm()  # Use as_dcm() for the rotation matrix

        # Create the transformation matrix
        transformation_matrix = np.eye(4)  # Start with an identity matrix
        transformation_matrix[:3, :3] = rotation  # Set the rotation part

        # Set the translation part (directly from the hoop's position)
        transformation_matrix[:3, 3] = [self.pose.position.x, self.pose.position.y, self.pose.position.z]

        # Transform the drone's position in the world frame to the hoop frame
        drone_world = np.array([pos.x, pos.y, pos.z, 1])  # [x, y, z, 1] for homogeneous coordinates
        drone_hoop = np.dot(np.linalg.inv(transformation_matrix), drone_world)  # Inverse transform

        return drone_hoop[:3]  # Return only the (x, y, z) coordinates

class HoopManager:
    def __init__(self, yaml_file_path):
        rospy.init_node("hoop_manager_node")
        self.hoops = self.load_hoops_from_yaml(yaml_file_path)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.drone_pose_sub = rospy.Subscriber('/drone/pose', Pose, self.drone_callback)

    def load_hoops_from_yaml(self, file_path):
        # YAML file has position of center in x,y,z and orientation roll, pitch, yaw for every hoop
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
    yaml_file_path = '/home/shaurya/need_for_pry/src/drone_race/config/track1.yaml' # Change YAML file path as needed
    hoop_manager = HoopManager(yaml_file_path)
    hoop_manager.publish_markers()
    rospy.spin()