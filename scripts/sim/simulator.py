#!/usr/bin/python3
import cv2
import numpy as np
# from tri import sonar_triangulation
# from anp import AnPAlgorithm
import random
random.seed(2)
import rospy
from cv_bridge import CvBridge
import sys


from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
from lias_anp.msg import SonarData  # 确保替换为你的包名
from std_msgs.msg import Header
import tf

from scipy.spatial.transform import Rotation as R
from utils import pose_to_transform_matrix

def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵"""
    return tf.transformations.quaternion_matrix(quaternion)[:3, :3]

def pose_to_transform_matrix_dict(pose):
    """将位姿转换为齐次变换矩阵"""
    position = pose['position']
    orientation = pose['orientation']
    
    # 提取平移向量
    translation = np.array([position['x'], position['y'], position['z']])
    
    # 提取四元数并转换为旋转矩阵
    quaternion = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # 构建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix

class Anp_sim:
    def __init__(self, manual_ctr=True):
        self.points_rviz = []
        self.xmin = 0
        self.xmax = 5
        self.ymin = -5
        self.ymax = 5
        self.zmin = -1
        self.zmax = 1
        
        for _ in range(100):
            x = random.uniform(self.xmin, self.xmax)
            y = random.uniform(self.ymin, self.ymax)
            z = random.uniform(self.zmin, self.zmax)
            self.points_rviz.append(Point(x, y, z))
       
        # self.points_rviz.append(Point(2, 0, 0))
        self.points = np.array([[point.x, point.y, point.z] for point in self.points_rviz])

        self.sonar_image = None
        self.pose = {'position': { 'x': 0.0, 'y': 0.0, 'z': 0.0,}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0,} }
        self.pose_T = pose_to_transform_matrix_dict(self.pose)
      
        self.estimated_pose = None
        
        # visualize
        self.img_match = None
        self.bridge = CvBridge()

        self.sonar_image_pub = rospy.Publisher('/sim/sonar_image', Image, queue_size=10)
        self.sonar_data_pub = rospy.Publisher('/sim/sonar_data_with_pose', SonarData, queue_size=10)

        self.sonar_marker_pub = rospy.Publisher('/rviz/sonar_view', Marker, queue_size=10)
        self.marker_pub = rospy.Publisher('/rviz/visualization_pts', Marker, queue_size=10)
        
        if manual_ctr:
            self.cmd_vel_sub = rospy.Subscriber('/joy/cmd_vel', Twist, self.cmd_vel_callback)
        else:
            self.sonar_pose_sub = rospy.Subscriber('/sonar_pose_publisher', PoseStamped, self.sonar_pose_callback)
        # self.traj_est_pub = rospy.Publisher("/trajectory_est", Path, queue_size=10)
        # self.traj_gt_pub = rospy.Publisher("/trajectory_gt", Path, queue_size=10)

        # Sonar
        # Define the sonar's field of view as a fan shape with top and bottom faces
        self.fov_horizontal = np.deg2rad(120)  # 90 degrees horizontal field of view
        self.fov_vertical = np.deg2rad(40)  # 60 degrees vertical field of view
        self.range_max = 5.0  # 5 meters range
        # Sonar image
        self.img_width, self.img_height = 500, 500
        self.s_p = None
        self.w_p = None
        self.si_q = None
        self.si_q_theta_Rho = None
        
        self.__timestep = 0
        
        self.__rviz_init()
    
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
        T_world = self.pose_T @ delta_T_robot
        
        # 限制平移部分在指定边界内
        T_world_t = T_world[:3, 3]
        T_world_t[0] = np.clip(T_world_t[0], -2, 2)
        T_world_t[1] = np.clip(T_world_t[1], -2, 2)
        T_world_t[2] = np.clip(T_world_t[2], self.zmin, self.zmax)
        # T_world_t[0] = np.clip(T_world_t[0], self.xmin, self.xmax)
        # T_world_t[1] = np.clip(T_world_t[1], self.ymin, self.ymax)
        # T_world_t[2] = np.clip(T_world_t[2], self.zmin, self.zmax)
        # 将调整后的平移部分放回 T_world 矩阵中
        T_world[:3, 3] = T_world_t
        
        # Update the pose transformation matrix
        self.pose_T = T_world

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
        
    def sonar_pose_callback(self, msg):

        # Create the translation vector from the linear velocities
        self.pose_T = pose_to_transform_matrix(msg.pose)

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
    
    def __points_in_fov(self):
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
        
        pts_in_fov_index = within_horizontal_fov & within_vertical_fov & within_range
        
        return pts_in_fov_index, self.pose
    
    def publish_sonar_image_and_data(self):
        # Get points that are within the FOV
        pts_in_fov_index, pose = self.__points_in_fov() # get index of pts in Fov stored in self.__pts_in_fov
        points_in_fov = self.points[pts_in_fov_index]        
        
        # Convert points to the sonar's coordinate frame
        sonar_position = np.array([self.pose['position']['x'], self.pose['position']['y'], self.pose['position']['z']])
        sonar_orientation = np.array([self.pose['orientation']['x'], self.pose['orientation']['y'], self.pose['orientation']['z'], self.pose['orientation']['w']])
        points_relative = points_in_fov - sonar_position
        r = R.from_quat(sonar_orientation)
        points_in_sonar_frame = r.inv().apply(points_relative)

        # Initialize a blank image
        image = np.ones((self.img_height, self.img_width,3), dtype=np.uint8) * 255
        center = (int(self.img_width/2), 0)
        radius = int(self.img_height)
        start_angle = -int(np.rad2deg(self.fov_horizontal)/2)  # 对称轴为中轴，扇形从-60度到60度
        end_angle = int(np.rad2deg(self.fov_horizontal)/2)
        cv2.ellipse(image, center, (radius, radius), 90, start_angle, end_angle, (0, 0, 0), -1)

        # Convert points to polar coordinates and map to image coordinates
        X, Y, Z = points_in_sonar_frame[:, 0], points_in_sonar_frame[:, 1], points_in_sonar_frame[:, 2]
        theta = np.arctan(Y/X)
        Rho = np.sqrt(X**2 + Y**2 + Z**2)

        ps_x = Rho * np.sin(theta)
        ps_y = Rho * np.cos(theta)
        
        si_q = []
        si_q_theta_Rho = []
        for x, y, theta_i, Rho_i in zip(ps_x, ps_y, theta, Rho):
            # Normalize to image coordinates
            x_img = int((x / self.range_max) * (self.img_width) + (self.img_width/2))
            y_img = int((y / self.range_max) * (self.img_height) )
            # image[y_img, x_img] = (255, 255, 255)
            cv2.circle(image, (x_img, y_img), 3, (255, 255, 255), -1)
            si_q.append(np.array([self.img_width/2-x_img, y_img]))
            si_q_theta_Rho.append(np.array([-theta_i, Rho_i]))
                
        # Convert to ROS Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.sonar_image_pub.publish(ros_image)
        
        self.w_p = points_in_fov
        self.s_p = points_in_sonar_frame
        self.si_q = np.array(si_q)
        self.si_q_theta_Rho = np.array(si_q_theta_Rho)
        
        # print(self.si_q_theta_Rho[0] - [-np.arctan(self.s_p[0][1]/self.s_p[0][0]), np.sqrt( np.sum(np.array(self.s_p[0])**2 ) )])
        # print(np.sum(np.array(self.s_p[0]))**2)
        
        # Publish SonarData with pose
        sonar_data_msg = SonarData()
        sonar_data_msg.header = Header()
        sonar_data_msg.header.stamp = rospy.Time.now()
        sonar_data_msg.indices = np.where(pts_in_fov_index)[0].tolist()
        sonar_data_msg.w_p = self.w_p.flatten()
        sonar_data_msg.s_p = self.s_p.flatten()
        sonar_data_msg.si_q = self.si_q.flatten()
        sonar_data_msg.si_q_theta_Rho = self.si_q_theta_Rho.flatten()
        sonar_data_msg.timestep = self.__timestep
        sonar_data_msg.pose.position.x = pose['position']['x']
        sonar_data_msg.pose.position.y = pose['position']['y']
        sonar_data_msg.pose.position.z = pose['position']['z']
        sonar_data_msg.pose.orientation.x = pose['orientation']['x']
        sonar_data_msg.pose.orientation.y = pose['orientation']['y']
        sonar_data_msg.pose.orientation.z = pose['orientation']['z']
        sonar_data_msg.pose.orientation.w = pose['orientation']['w']
        # indices = np.where(pts_in_fov_index)[0]
        # print(indices)

        self.sonar_data_pub.publish(sonar_data_msg)

    def main_process(self, step=1):        
        rospy.init_node('anp_sim')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_sonar_image_and_data()
            self.__visualize()
            self.__timestep += 1
            rate.sleep()
            
    def __visualize(self):
        self.__publish_points()
        self.__publish_sonar_view()
        return
    
    def __rviz_init(self):
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

    def __publish_sonar_view(self):
        """
        publish sonar in rviz       
        """
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
    
    def __publish_points(self):
        """
        publish points in rviz       
        """
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
    manual_ctr = True
    if len(sys.argv) != 2 or sys.argv[1] not in ['-m', '-a']:
        print("Usage: rosrun lias_anp simulator.py -m/-a (manual control by default)")
    elif sys.argv[1] != '-m':
        manual_ctr = False
    estimator = Anp_sim(manual_ctr)
    estimator.main_process()
