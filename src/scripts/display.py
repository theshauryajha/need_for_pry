#!/usr/bin/env python3
import rospy
import rospkg
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
import yaml

class DisplayImage:
    def __init__(self):
        self.publisher = rospy.Publisher('/camera/image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)  # Publishing rate at 30 Hz

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drone_race')
        track = rospy.get_param('track_config')
        yaml_file_path = os.path.join(package_path, 'config', track, 'leaderboard.yaml')

        self.load_leaderboard(yaml_file_path)
        self.start_time = rospy.Time.now()
        self.lap_start_time = None

    def load_leaderboard(self, filename):
        with open(filename, 'r') as file:
            self.leaderboard = yaml.safe_load(file) or {}

    def render_display(self, items, image_width=1280, image_height=720):
        image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

        total_height = 0
        max_width = 0
        font_sizes = []

        # Calculate initial sizes
        for text, metadata in items:
            font = metadata.get('font', cv2.FONT_HERSHEY_SIMPLEX)
            initial_font_scale = 1
            thickness = metadata.get('thickness', 2)

            (text_width, text_height), _ = cv2.getTextSize(text, font, initial_font_scale, thickness)
            total_height += text_height + 20  # Add padding
            max_width = max(max_width, text_width)
            font_sizes.append((text_width, text_height, initial_font_scale))

        # Calculate scaling factor
        width_scale = (image_width * 0.9) / max_width
        height_scale = (image_height * 0.9) / total_height
        scale = min(width_scale, height_scale)

        y_offset = (image_height - total_height * scale) // 2

        for (text, metadata), (text_width, text_height, initial_font_scale) in zip(items, font_sizes):
            font = metadata.get('font', cv2.FONT_HERSHEY_SIMPLEX)
            font_scale = initial_font_scale * scale
            thickness = max(1, int(metadata.get('thickness', 2) * scale))
            color = metadata.get('color', (255, 255, 255))

            (scaled_width, scaled_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            x = (image_width - scaled_width) // 2
            y = int(y_offset + scaled_height)

            # Add a subtle shadow effect
            shadow_color = tuple(max(0, c - 100) for c in color)
            cv2.putText(image, text, (x+2, y+2), font, font_scale, shadow_color, thickness, cv2.LINE_AA)

            # Draw the main text
            cv2.putText(image, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

            y_offset += int(scaled_height + 20 * scale)

        # Add a subtle vignette effect
        vignette = np.zeros((image_height, image_width), dtype=np.uint8)
        cv2.circle(vignette, (image_width//2, image_height//2), int(min(image_width, image_height)*0.7), (255, 255, 255), -1)
        vignette = cv2.GaussianBlur(vignette, (55, 55), 0)
        image = cv2.addWeighted(image, 1, cv2.cvtColor(vignette, cv2.COLOR_GRAY2BGR), 0.2, 0)

        return image

    def create_image(self):
        now = rospy.Time.now()
        elapsed_time = (now - self.start_time).to_sec()

        items = []

        if elapsed_time < 9:
            if elapsed_time < 1:
                items.append(("Welcome to the drone race game!", {'color': (0, 255, 255)}))
            
            elif elapsed_time < 3:
                items.append(("Find your position on the leaderboard!", {'color': (255, 255, 0)}))
                items.append(("Leaderboard", {'color': (255, 255, 255)}))
                for player, time in self.leaderboard.items():
                    items.append((f"{player} || {time}", {'color': (200, 200, 200)}))

            elif elapsed_time < 4:
                items.append(("Let's fly!", {'color': (0, 255, 0)}))
            
            else:
                countdown = 9 - int(elapsed_time)
                if countdown > 0:
                    items.append((str(countdown), {'color': (255, 0, 0)}))
                else:
                    items.append(("!! GO !!", {'color': (0, 255, 0)}))

        if elapsed_time >= 9:
            if self.lap_start_time is None:
                self.lap_start_time = rospy.Time.now()
            lap_time = (now - self.lap_start_time).to_sec()
            items.append((f"Lap Time: {lap_time:.3f}", {'color': (0, 255, 0)}))

        return self.render_display(items)

    def publish_image(self):
        while not rospy.is_shutdown():
            try:
                image = self.create_image()
                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.publisher.publish(ros_image)
                self.rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Node interrupted")
                break
            except Exception as e:
                rospy.logerr(f"Error in publish_image: {str(e)}")

if __name__ == '__main__':
    try:
        rospy.init_node('image_display_node')
        display_image = DisplayImage()
        display_image.publish_image()
    except rospy.ROSInterruptException:
        pass