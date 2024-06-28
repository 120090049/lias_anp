import cv2
import os
import numpy as np
# from tri import sonar_triangulation
from utils import pose_to_transform_matrix
# from anp import AnPAlgorithm
import random
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Anp_sim:
    def __init__(self):
        rospy.init_node('anp_sim', anonymous=True)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rangeX = (1.6, 2.8)
        self.rangeY = (-0.6, 0.6)
        self.rangeZ = (-0.3, 0.3)
        self.points = []
        self.pose = {
            'position': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 0.0,
            }
        }

    def generate_random_points(self):
        for _ in range(100):
            x = random.uniform(*self.rangeX)
            y = random.uniform(*self.rangeY)
            z = random.uniform(*self.rangeZ)
            self.points.append(Point(x, y, z))

    def publish_pose(self):
        # 示例：发布当前的pose
        rospy.loginfo(f"Publishing pose: {self.pose}")

    def publish_points(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in self.points:
            marker.points.append(point)

        self.marker_pub.publish(marker)

    def run(self):
        self.generate_random_points()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.publish_points()
            self.publish_pose()
            rate.sleep()

if __name__ == '__main__':
    try:
        anp_sim = Anp_sim()
        anp_sim.run()
    except rospy.ROSInterruptException:
        pass
