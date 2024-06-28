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
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion, Twist
from visualization_msgs.msg import Marker

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

from scipy.spatial.transform import Rotation as R

# left right corner is (0,0) bottom right (50,100)
def convert_points_to_polar(points, width, height): # 512 # 798
    # 参考点(center)
    center = (width / 2, height)

    # 提取x和y坐标
    x_coords = -points[:, 0, 0] + width
    y_coords = points[:, 0, 1]

    # 计算距离(distance)
    distances = 20/height*np.sqrt((x_coords - center[0])**2 + (y_coords - center[1])**2)

    # 计算角度(Theta)（以弧度表示）
    thetas = np.arctan2(center[1] - y_coords, x_coords - center[0]) - np.pi/2

    return thetas, distances


class Anp_sim:
    def __init__(self):
        self.points_rviz = []

        for _ in range(100):
            x = random.uniform(-5, 5)
            y = random.uniform(-5, 5)
            z = random.uniform(-1, 1)
            self.points_rviz.append(Point(x, y, z))
        self.points = np.array([[point.x, point.y, point.z] for point in self.points_rviz])

        self.sonar_image = None
        self.pose = {'position': { 'x': 0.0, 'y': 0.0, 'z': 0.0,}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0,} }
        self.pose_T = pose_to_transform_matrix(self.pose)
      
        self.estimated_pose = None
        
        # visualize
        self.img_match = None
        self.bridge = CvBridge()
        self.image_pub_1 = rospy.Publisher("/sonar_image", Image, queue_size=10)
        # self.image_pub_2 = rospy.Publisher("/selected_matches", Image, queue_size=10)
        # self.pose_est_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        self.sonar_marker_pub = rospy.Publisher('/sonar_view', Marker, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_pts', Marker, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/joy/cmd_vel', Twist, self.cmd_vel_callback)
        # self.traj_est_pub = rospy.Publisher("/trajectory_est", Path, queue_size=10)
        # self.traj_gt_pub = rospy.Publisher("/trajectory_gt", Path, queue_size=10)

        # Initialize the sonar view marker
        # Define the sonar's field of view as a fan shape with top and bottom faces
        self.fov_horizontal = np.deg2rad(60)  # 90 degrees horizontal field of view
        self.fov_vertical = np.deg2rad(10)  # 60 degrees vertical field of view
        self.range_max = 2.0  # 5 meters range

        self.__pts_in_fov_index = None
        
        self.rviz_init()
    
    def rviz_init(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "sonar"
        self.marker.id = 0
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.a = 0.3  # Transparency
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

     
        # Define the vertices of the fan shape
        origin = Point(0, 0, 0)
        angle_step = np.deg2rad(5)  # Angle step for the fan shape

        # Vertices for the bottom and top fan shapes
        bottom_points = []
        top_points = []

        angle_min = -self.fov_horizontal / 2
        angle_max = self.fov_horizontal / 2
        angle_increment = (angle_max - angle_min) / (self.fov_horizontal / angle_step)

        for i in range(int(self.fov_horizontal / angle_step) + 1):
            angle = angle_min + i * angle_increment
            x = self.range_max * np.cos(angle)
            y = self.range_max * np.sin(angle)
            z_bottom = -self.range_max * np.tan(self.fov_vertical / 2)
            z_top = self.range_max * np.tan(self.fov_vertical / 2)
            bottom_points.append(Point(x, y, z_bottom))
            top_points.append(Point(x, y, z_top))

        # Create triangles for the bottom and top fan shapes and connect them
        for i in range(len(bottom_points) - 1):
            # Bottom fan triangles
            self.marker.points.append(origin)
            self.marker.points.append(bottom_points[i])
            self.marker.points.append(bottom_points[i + 1])

            # Top fan triangles
            self.marker.points.append(origin)
            self.marker.points.append(top_points[i])
            self.marker.points.append(top_points[i + 1])

            # Side triangles
            # First triangle
            self.marker.points.append(bottom_points[i])
            self.marker.points.append(top_points[i])
            self.marker.points.append(bottom_points[i + 1])

            # Second triangle
            self.marker.points.append(bottom_points[i + 1])
            self.marker.points.append(top_points[i])
            self.marker.points.append(top_points[i + 1])
        
    def cmd_vel_callback(self, msg):
        dt = 1.0 / 100.0  # Assuming the loop runs at 10 Hz

        # Create the translation vector from the linear velocities
        translation = np.array([msg.linear.x * dt, msg.linear.y * dt, msg.linear.z * dt])

        # Create the rotation matrix from the angular velocities
        delta_rotation_matrix = tf.transformations.euler_matrix(
            msg.angular.x * dt,
            msg.angular.y * dt,
            msg.angular.z * dt
        )

        # Combine translation and rotation into a transformation matrix
        delta_T_robot = tf.transformations.compose_matrix(translate=translation) @ delta_rotation_matrix
        delta_T_world = self.pose_T @ delta_T_robot
        # Update the pose transformation matrix
        self.pose_T = delta_T_world

        # Extract the updated position and orientation from the transformation matrix
        updated_translation = tf.transformations.translation_from_matrix(self.pose_T)
        updated_orientation = tf.transformations.quaternion_from_matrix(self.pose_T)

        # Update the pose dictionary
        self.pose['position']['x'] = updated_translation[0]
        self.pose['position']['y'] = updated_translation[1]
        self.pose['position']['z'] = updated_translation[2]
        self.pose['orientation']['x'] = updated_orientation[0]
        self.pose['orientation']['y'] = updated_orientation[1]
        self.pose['orientation']['z'] = updated_orientation[2]
        self.pose['orientation']['w'] = updated_orientation[3]
    
        
    def points_in_fov(self):
        """
        Determine which points are within the field of view (FOV) of a sonar.

        Returns:
        np.ndarray: A boolean array of length N indicating which points are within the FOV.
        """
        sonar_position = np.array([self.pose['position']['x'], self.pose['position']['y'], self.pose['position']['z']])
        sonar_orientation = np.array([self.pose['orientation']['x'], self.pose['orientation']['y'], self.pose['orientation']['z'], self.pose['orientation']['w']])
        
        # Convert points to the sonar's coordinate frame
        points_relative = self.points - sonar_position
        r = R.from_quat(sonar_orientation)
        points_in_sonar_frame = r.inv().apply(points_relative)

        # Calculate angles and distances in the sonar's coordinate frame
        x, y, z = points_in_sonar_frame[:, 0], points_in_sonar_frame[:, 1], points_in_sonar_frame[:, 2]
        distances = np.sqrt(x**2 + y**2 + z**2)
        horizontal_angles = np.arctan2(y, x)
        vertical_angles = np.arctan2(z, np.sqrt(x**2 + y**2))

        # Check if self.points are within the FOV and range
        within_horizontal_fov = np.abs(horizontal_angles) <= (self.fov_horizontal / 2)
        within_vertical_fov = np.abs(vertical_angles) <= (self.fov_vertical / 2)
        within_range = distances <= self.range_max
        
        self.__pts_in_fov_index = within_horizontal_fov & within_vertical_fov & within_range
        
    
    def publish_sonar_image(self):
        # Initialize a blank image
        img_width, img_height = 500, 500
        image = np.zeros((img_height, img_width), dtype=np.uint8)

        # Map points to image coordinates
        for point in self.__points_in_fov:
            x_img = int((point[0] / 2.0) * img_width / 2 + img_width / 2)
            y_img = int((point[1] / 2.0) * img_height / 2 + img_height / 2)
            if 0 <= x_img < img_width and 0 <= y_img < img_height:
                image[y_img, x_img] = 255

        # Convert to ROS Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
        self.sonar_image_pub.publish(ros_image)

    def main_process(self, step=1):
        rospy.init_node('anp_sim')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.points_in_fov() # get index of pts in Fov stored in self.__pts_in_fov
            self.visualize()
            rate.sleep()
            
    def visualize(self):
        # self.publish_images()
        # self.publish_traj_gt()
        self.publish_points()
        self.publish_sonar_view()
        return
    
    def publish_pose(self):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = "map"

        pose_stamped_msg.pose.position = Point(
            self.pose['position']['x'],
            self.pose['position']['y'],
            self.pose['position']['z']
        )
        pose_stamped_msg.pose.orientation = Quaternion(
            self.pose['orientation']['x'],
            self.pose['orientation']['y'],
            self.pose['orientation']['z'],
            self.pose['orientation']['w']
        )
        
        self.pose_est_pub.publish(pose_stamped_msg)
    
    def publish_sonar_view(self):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position = Point(
            self.pose['position']['x'],
            self.pose['position']['y'],
            self.pose['position']['z']
        )
        self.marker.pose.orientation = Quaternion(
            self.pose['orientation']['x'],
            self.pose['orientation']['y'],
            self.pose['orientation']['z'],
            self.pose['orientation']['w']
        )
        self.sonar_marker_pub.publish(self.marker)
    
    def publish_points(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02
        marker.scale.y = 0.02

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point in self.points_rviz:
            marker.points.append(point)

        self.marker_pub.publish(marker)


if __name__ == '__main__':
    estimator = Anp_sim()
    estimator.main_process()
