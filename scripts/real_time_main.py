#!/usr/bin/python3

"""
This code is to analyze in which case the determiant of 
triangularation method objective function will decrease to 0 

It will subscribe the "/sim/sonar_data_with_pose" rostopic 
and calculate the determinant between the 1st and the last frame

So you need to run simulator first
"""
# Append the root dir
import sys, roslib, os
project_root = roslib.packages.get_pkg_dir('lias_anp')
root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
sys.path.append(root_dir)

import numpy as np
from lias_anp.msg import SonarData  # 确保替换为你的包名
import csv
import rospy
import copy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import threading

# This is the function implement in this package
from utils.match_pairs import get_match_pairs
from anp.anp_alg import AnPAlgorithm
from utils.pose2matrix import pose_to_transform_matrix, transform_matrix_to_ros_pose
from utils.coordinate_system_transform import coordinate_transform_Pose, coordinate_transform_Pose_back, coordinate_transform_pt, coordinate_transform_pt_back
from tri.tri import ANRS, GTRS, gradient_descent

from visualization_msgs.msg import Marker



class Odometer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tri_sim', anonymous=True)
        
        # 定义订阅者
        self.sonar_data_sub = rospy.Subscriber('/sim/sonar_data_with_pose', SonarData, self.sonar_callback)
        # self.pose_sub = rospy.Subscriber('/sim/sonar_data_with_pose', PoseStamped, self.pose_callback)
        self.traj_gt_pub = rospy.Publisher("/rviz/estimated_trajectory", Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/rviz/visualize_triangulated_pts', Marker, queue_size=10)

        # 定义数据存储
        self.w_p = None
        self.s_p = None
        self.si_q = None
        self.timestep = None
        
        self.pts_indice = None
        self.si_q_theta_Rho = None
        
        self.pts_indice_prime = None
        self.si_q_theta_Rho_prime = None
        
        self.pose = None
        self.pose_T = None
        self.R = None
        self.t = None
        
        self.odometry = []
        self.P_dict = {} # dictionary to store the points in the world frame
        self.P_dict_lock = threading.Lock()
        
        self.anp_algorithm = AnPAlgorithm()
        
        
        # 初始化标志
        self.initialization_iteration = 0
        self.get_the_initial_pose = False
        self.initialized = False
    
    def initialize_first_frame(self, data):
        """ We need to xxxx """
        self.initialization_iteration += 1
        if not self.get_the_initial_pose:
            ## NEED TO BE CHANGE IN REAL EXPERIMENT (we need to define the initial position in the world frame)
            initial_pose = coordinate_transform_Pose(pose_to_transform_matrix(data.pose))
            self.pose_T0 = initial_pose
            self.si_q_theta_Rho_T0 = np.array(data.si_q_theta_Rho).reshape(-1, 2)
            self.pts_indice_T0 = np.array(data.indices)
            self.get_the_initial_pose = True
            # self.origin = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
            self.odometry.append(transform_matrix_to_ros_pose(coordinate_transform_Pose_back( self.pose_T0 )) )
        
        else:
            # During the initialization process, we need to know the pose ground truth
            Tri_T0_gt = self.pose_T0
            self.pose_T1 = coordinate_transform_Pose(pose_to_transform_matrix(data.pose))
            Tri_T1_gt = self.pose_T1
            Tri_T_gt = np.linalg.inv(Tri_T1_gt) @ Tri_T0_gt
            
            # Here is information we get from sonar 
            self.si_q_theta_Rho_T1 = np.array(data.si_q_theta_Rho).reshape(-1, 2)
            self.pts_indice_T1 = np.array(data.indices)
            theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(self.si_q_theta_Rho_T0, self.pts_indice_T0, self.si_q_theta_Rho_T1, self.pts_indice_T1)
            
            # Here we do the triangularation
            for i in range(len(theta_Rho)):
                s_P_init, determinant = ANRS(Tri_T_gt, theta_Rho[i], theta_Rho_prime[i])
                # s_P_init = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                if s_P_init is not None:
                    s_P, good_reconstruct = gradient_descent(s_P_init, theta_Rho[i], theta_Rho_prime[i], Tri_T_gt)
                    if good_reconstruct:
                        w_P = ( self.pose_T0 @ np.hstack([s_P, 1]) )[:3]
                        key = common_indices[i]
                        if key not in self.P_dict:
                            self.P_dict[key] = w_P                
            
            self.odometry.append(transform_matrix_to_ros_pose(coordinate_transform_Pose_back( self.pose_T1 )) )
            # rospy.loginfo("dictor")
            # print(len(self.P_dict))
            if len(self.P_dict) > 15:
                self.initialized = True
                rospy.loginfo("Successfully initialized after {} iteration, now we have {} 3d feature points".format(self.initialization_iteration, len(self.P_dict)))
             
    def sonar_callback(self, data):
        # if len(data.indices) > 0:
        with self.P_dict_lock:
            # Points ground truth
            w_P_gt = np.array(data.w_p).reshape(-1, 3)
            self.w_P_gt = coordinate_transform_pt( w_P_gt.T ).T
            
            if not self.initialized:
                self.initialize_first_frame(data)
                
            else:
                theta_Rho2 = np.array(data.si_q_theta_Rho).reshape(-1, 2)
                pts_indice2 = np.array(data.indices)
                
                # self.w_p = np.array(data.w_p).reshape(-1, 3)
                # self.s_p = np.array(data.s_p).reshape(-1, 3)
                # self.si_q = np.array(data.si_q).reshape(-1, 2)
                # self.timestep = data.timestep
                
                ##################################################
                # ANP
                ## Get q_si2 and P_w for ANP
                q_si_x2 = np.sin(theta_Rho2.T[0]) * theta_Rho2.T[1]
                q_si_y2 = np.cos(theta_Rho2.T[0]) * theta_Rho2.T[1]
                q_si2 = np.vstack([q_si_x2, q_si_y2])

                ##  Find matching pairs of q_si and and w_P in dictionary
                filtered_P_w_values = []
                filtered_q_si_index = []
                for j, idx in enumerate( pts_indice2 ):
                    value = self.P_dict.get(idx)
                    if value is not None:
                        filtered_P_w_values.append(value[:3])
                        filtered_q_si_index.append(j)
                q_si2 = q_si2.T[filtered_q_si_index].T
                P_w = np.array(filtered_P_w_values).T
                t_s_cal, R_sw_cal = self.anp_algorithm.compute_t_R(q_si2, P_w)
                T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
                T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
                T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
                T2 = np.linalg.inv(T2)
                calculated_T2 = copy.deepcopy(T2)
                
                Tri_T1 = self.pose_T1 # pose in first frame for triangluaration
                # Update pose
                self.pose_T1 = T2
                
                # END ANP
                #####################################################
                
                # TRI
                
                self.si_q_theta_Rho_T2 = theta_Rho2
                self.pts_indice_T2 = pts_indice2
                theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(self.si_q_theta_Rho_T1, self.pts_indice_T1, self.si_q_theta_Rho_T2, self.pts_indice_T2)
            
                Tri_T2 = calculated_T2
                T_matrix = np.linalg.inv(Tri_T2) @ Tri_T1

                reconstrubtion_error_list = []
                for i in range(len(theta_Rho)):
                    s_P_init, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                    if s_P_init is not None:
                        s_P, good_reconstruct = gradient_descent(s_P_init, theta_Rho[i], theta_Rho_prime[i], T_matrix)
                        if good_reconstruct:
                            w_P = ( Tri_T1 @ np.hstack([s_P, 1]) )[:3]
                            key = common_indices[i]
                            
                            print(self.P_dict[key], w_P)
                            difference = np.linalg.norm( w_P - w_P_gt[i] )
                            reconstrubtion_error_list.append(difference)
                            if key not in self.P_dict and difference < 0.1:
                                self.P_dict[key] = w_P
                                #############################
                                #############################
                                #############################
                                #############################
                        # P_dict[key] = w_P
                count = sum(1 for error in reconstrubtion_error_list if error > 0.1)
                print("Outlier count number: ", count)   
                
                self.odometry.append(transform_matrix_to_ros_pose( coordinate_transform_Pose_back(self.pose_T1) ))
                # reconstrubtion_error_list = []
                # for index, triangulated_point in self.P_dict.items():
                #     print(triangulated_point, len(w_P_gt), index)
                #     difference = np.linalg.norm( triangulated_point - w_P_gt[index] )
                #     reconstrubtion_error_list.append(difference)
                # count = sum(1 for error in reconstrubtion_error_list if error > 0.1)
                # print("Outlier count number: ", count)   

                # TRI END
                ####################################################
    
    def __publish_estimated_traj(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        current_time = rospy.Time.now()
        for i, pose in enumerate(self.odometry):
            pose_stamped = PoseStamped()
            # 使用递增的时间戳
            pose_stamped.header.stamp = current_time + rospy.Duration(i * 0.1)  # 每个姿态相隔0.1秒
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = pose['position']['x']
            pose_stamped.pose.position.y = pose['position']['y']
            pose_stamped.pose.position.z = pose['position']['z']+0.01
            pose_stamped.pose.orientation.x = pose['orientation']['x']
            pose_stamped.pose.orientation.y = pose['orientation']['y']
            pose_stamped.pose.orientation.z = pose['orientation']['z']
            pose_stamped.pose.orientation.w = pose['orientation']['w']

            path_msg.poses.append(pose_stamped)
        self.traj_gt_pub.publish(path_msg)
    
    def __publish_triangularized_pts(self):
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

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        with self.P_dict_lock:
            for index, point in self.P_dict.items():
                point = coordinate_transform_pt_back(point)
                marker.points.append(Point(point[0], point[1], point[2]+0.01))

        self.marker_pub.publish(marker)
    
    def loop(self):        
        # 打开一个文件用于写入数据
        with open(root_dir+'/record/experiment_recorder.csv', mode='a', newline='') as file:
            self.writer = csv.writer(file)
            # 写入表头
            # self.writer.writerow(['s_P_0', 'determinant0', 's_P_1', 'determinant1'])

            rate = rospy.Rate(50)
            while not rospy.is_shutdown():
                self.__publish_estimated_traj()
                self.__publish_triangularized_pts()
                rate.sleep()

# 确保你的脚本被执行时调用loop方法
if __name__ == '__main__':
    odo = Odometer()
    odo.loop()
