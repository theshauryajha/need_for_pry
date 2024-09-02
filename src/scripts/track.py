#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import math
import tf.transformations as transformations

class Hoop:
    def __init__(self, pose, radius, hoop_id):
        self.pose = pose
        self.radius = radius
        self.cleared = False
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

        return marker

    def is_collision(self, drone_pos):
        dy = drone_pos.y - self.pose.position.y
        dz = drone_pos.z - self.pose.position.z
        distance = (dy**2 + dz**2)**0.5
        return distance < self.radius

class HoopManager:
    def __init__(self, yaml_file_path):
        rospy.init_node("hoop_manager_node")
        self.hoops = self.load_hoops_from_yaml(yaml_file_path)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.drone_pose_sub = rospy.Subscriber('/drone/pose', Pose, self.drone_callback)

    def load_hoops_from_yaml(self, file_path):
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
    yaml_file_path = '/home/shaurya/need_for_pry/src/drone_race/config/track1.yaml'
    hoop_manager = HoopManager(yaml_file_path)
    hoop_manager.publish_markers()
    rospy.spin()