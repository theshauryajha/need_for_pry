#!/usr/bin/env python3
import rospy
import rospkg
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime, timedelta
import yaml

class DisplayImage:
    def __init__(self):
        # Initialize the ROS publisher
        self.publisher = rospy.Publisher('/camera/image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)  # Publishing rate at 30 Hz

        # Load leaderboard for track
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drone_race')
        track = rospy.get_param('track_config')
        yaml_file_path = os.path.join(package_path, 'config', track, 'leaderboard.yaml')

        self.load_leaderboard(yaml_file_path)

        # Initialize times
        self.start_time = datetime.now()
        self.messages = [
            (0, 1.5, "Welcome to the drone race game!"),
            (1.5, 2.5, "Find your position on the leaderboard!"),
            (2.5, 3, "Let's fly!"),
            (3, 4, "5"),
            (4, 5, "4"),
            (5, 6, "3"),
            (6, 7, "2"),
            (7, 8, "1"),
            (8, 9, "!! GO !!")
        ]
        self.lap_start_time = None
    
    def load_leaderboard(self, filename):
        with open(filename, 'r') as file:
            self.leaderboard = yaml.safe_load(file) or {}

    def draw_text(self, image, text, position, font_scale, thickness, color):
        font = cv2.FONT_HERSHEY_SIMPLEX
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x = position[0] - text_width // 2
        y = position[1] + text_height // 2
        cv2.putText(image, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

    def draw_leaderboard(self, image):
        y0, dy = 100, 30
        x0 = image.shape[1] - 300
        self.draw_text(image, "Leaderboard", (x0 + 150, y0 - 40), 1.5, 2, (255, 255, 255))
        for idx, (player, time) in enumerate(self.leaderboard.items()):
            self.draw_text(image, f"{player} || {time}", (x0 + 150, y0 + idx * dy), 1, 2, (255, 255, 255))

    def create_image(self):
        image_width = 1280
        image_height = 720
        image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
        
        now = datetime.now()
        elapsed_time = (now - self.start_time).total_seconds()
        rospy.loginfo(f"Elapsed time: {elapsed_time}")
        
        # Determine which message to display
        for start, end, message in self.messages:
            if start <= elapsed_time < end:
                self.draw_text(image, message, (image_width // 2, image_height // 2), 1.5, 2, (255, 255, 255))
                break
        
        if elapsed_time >= 2.5:
            self.draw_leaderboard(image)
        
        if elapsed_time >= 9:
            if self.lap_start_time is None:
                self.lap_start_time = rospy.Time.now()
            elapsed = (rospy.Time.now() - self.lap_start_time).to_sec()
            self.draw_text(image, f"Lap Time: {elapsed:.3f}", (image_width // 2, image_height // 2 + 150), 2, 3, (0, 255, 0))
        
        return image
    
    def publish_image(self):
        while not rospy.is_shutdown():
            # Create an image with the current timestamp
            image = self.create_image()
            
            # Convert the image to a ROS message
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            
            # Publish the image
            self.publisher.publish(ros_image)
            
            # Sleep to maintain the rate
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('image_display_node')
    display_image = DisplayImage()
    display_image.publish_image()