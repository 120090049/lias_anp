import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import PoseArray, Pose
import tf
from nav_msgs.msg import Path
from read_record import read_csv_file

class TrajectoryVisualizer:
    
    def __init__(self, trajectory, points, trajectory_rgb=[1,0,0], points_rgb=[1,0,0], label=0):
        self.label = str(label)
        self.trajectory = trajectory
        self.trajectory_color = trajectory_rgb
        self.points = points
        self.points_color = points_rgb
        
        
        # self.pose_pub = rospy.Publisher('/rviz/visualized_pose'+self.label, PoseStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/set_sonar_pose', PoseStamped, queue_size=10)
        self.trajectory_pub = rospy.Publisher("/rviz/trajectory"+self.label, Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/rviz/tri_pts'+self.label, Marker, queue_size=10)

        self.__timestep = 1
        
    def main_process(self):        
        node_name = "case_record_" + self.label
        rospy.init_node(node_name)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.__publish_fov_pts()
            self.__publish_trajectory()
            self.__timestep += 1
            rate.sleep()
    
    def __publish_fov_pts(self):
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

        marker.color.r = self.points_color[0]
        marker.color.g = self.points_color[1]
        marker.color.b = self.points_color[2]
        marker.color.a = 1.0
        
        for point in self.points[self.__timestep]:
            marker.points.append(Point(point[0], point[1], point[2]))

        self.marker_pub.publish(marker)
    
    def __publish_trajectory(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        current_time = rospy.Time.now()
        for i, pose in enumerate(self.trajectory[:self.__timestep]):
            pose_stamped = PoseStamped()
            # 使用递增的时间戳
            pose_stamped.header.stamp = current_time + rospy.Duration(i * 0.1)  # 每个姿态相隔0.1秒
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = pose[0,3]
            pose_stamped.pose.position.y = pose[1,3]
            pose_stamped.pose.position.z = pose[2,3]
            path_msg.poses.append(pose_stamped)
        self.trajectory_pub.publish(path_msg)
        
        last_pose = self.trajectory[self.__timestep - 1]
        last_pose_stamped = PoseStamped()
        last_pose_stamped.header.stamp = rospy.Time.now()
        last_pose_stamped.header.frame_id = "map"
        last_pose_stamped.pose.position.x = last_pose[0,3]
        last_pose_stamped.pose.position.y = last_pose[1,3]
        last_pose_stamped.pose.position.z = last_pose[2,3]
        
        # 从旋转矩阵提取四元数
        last_quat = tf.transformations.quaternion_from_matrix(last_pose)
        last_pose_stamped.pose.orientation.x = last_quat[0]
        last_pose_stamped.pose.orientation.y = last_quat[1]
        last_pose_stamped.pose.orientation.z = last_quat[2]
        last_pose_stamped.pose.orientation.w = last_quat[3]
        
        self.pose_pub.publish(last_pose_stamped)




# Usage
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/record/atraj.csv"  
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/app/record2/atraj.csv"  
file_path = "/home/clp/catkin_ws/src/lias_anp/record/nonapp/record2/atraj.csv"  
real_poses1, estimated_poses_anp, coordinates_list = read_csv_file(file_path)


plotter = TrajectoryVisualizer(real_poses1, coordinates_list, 0)


# Plot all the added trajectories
plotter.main_process()