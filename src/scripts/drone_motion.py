#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
import tf2_ros
import rospkg
import os

class DroneMotion:
    def __init__(self):
        rospy.init_node('drone_motion_node')
        
        self.pose_pub = rospy.Publisher('/drone/pose', Pose, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        self.velocity_sub = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)

        self.gravity = -2.0
        self.position = [-5, 0, 1]  # [x, y, z]
        self.orientation = [0, 0, 0, 1]  # [x, y, z, w] (quaternion)
        self.velocity = [0, 0, 0] # [x, y, z]
        
        self.rate = rospy.Rate(50)  # 50 Hz
        
        # Initialize tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(1.0/50), self.update_callback)

    def update_callback(self, event):
        dt = 1.0 / 50

        # Apply gravity to z-velocity
        self.velocity[2] += self.gravity * dt

        # Update position based on current velocity
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt

        # Stop at ground level
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = 0

    def velocity_callback(self, msg):
        # Update velocity based on received Twist message
        self.velocity[0] = msg.linear.x
        self.velocity[1] = msg.linear.y
        self.velocity[2] += msg.linear.z  # Add to existing z velocity (affected by gravity)
        
        # Update orientation based on angular velocity (simplified)
        yaw = msg.angular.z * (1.0 / 50)  # Assuming 50 Hz update rate
        q = quaternion_from_euler(0, 0, yaw)
        self.orientation = [q[0], q[1], q[2], q[3]]

    def publish_pose_and_joint_states(self):
        # Publish Pose
        pose_msg = Pose()
        pose_msg.position.x = self.position[0]
        pose_msg.position.y = self.position[1]
        pose_msg.position.z = self.position[2]
        pose_msg.orientation.x = self.orientation[0]
        pose_msg.orientation.y = self.orientation[1]
        pose_msg.orientation.z = self.orientation[2]
        pose_msg.orientation.w = self.orientation[3]
        self.pose_pub.publish(pose_msg)

        # Publish Joint States (empty for this URDF as it doesn't have movable joints)
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(joint_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_pose_and_joint_states()
            self.rate.sleep()

if __name__ == '__main__':
    '''
        Assume the start position is always the origin
        Subscribe to the topic from teleop-twist-keyboard
        With every velocity update, integrate it to find the position of the drone 
        Publish the current position of the drone us tf
    ''' 
    try:
        drone_motion = DroneMotion()
        drone_motion.run()
    except rospy.ROSInterruptException:
        pass