#!/usr/bin/python3
import numpy as np
import transforms3d


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import csv
import os

from matplotlib import cm

# Append the root dir
import sys, roslib, os
lias_anp_dir = roslib.packages.get_pkg_dir('lias_anp')
scripts_dir = os.path.abspath(os.path.join(lias_anp_dir, 'scripts'))
sys.path.append(scripts_dir)
from utils.sonar_data_processor import SonarDataReader
from utils.match_pairs import get_match_pairs
from anp.anp_alg import AnPAlgorithm
from tri.tri import ANRS, GTRS, gradient_descent
from utils.pose2matrix import pose_to_transform_matrix, ros_pose_to_transform_matrix, transform_matrix_to_ros_pose
from utils.coordinate_system_transform import coordinate_transform_Pose, coordinate_transform_Pose_back, coordinate_transform_pt, coordinate_transform_pt_back
from utils.transformation_matrix_add_noise import add_noise_to_pose

import yaml
yaml_file_path = os.path.join(lias_anp_dir, 'yaml/odom.yaml')
with open(yaml_file_path, 'r') as file:
    params = yaml.safe_load(file)
    RECONSTRUCTION_ERROR_THRESHOLD = params['RECONSTRUCTION_ERROR_THRESHOLD']
    RECORD = params['RECORD']
    DATA_PATH = params['data_path']
    ANP_METHOD = params['ANP_METHOD']

sonar_data_dir = str(lias_anp_dir) + DATA_PATH
reord_dir = str(lias_anp_dir) + "/record/" + ANP_METHOD
reader = SonarDataReader(filepath = sonar_data_dir)
reader.read_data()
data = reader.get_data()

from functools import reduce
def intersect_multiple_arrays(array_list):
    array_lists = [list(arr) for arr in array_list]
    return list(reduce(np.intersect1d, array_lists))

def get_match_pairs(theta_Rhos_across_times, pts_indices_across_times):
    common_indices = intersect_multiple_arrays(pts_indices_across_times)
    matched_theta_Rho_across_times = [] 
    for pts_indicei, theta_Rhoi in zip(pts_indices_across_times, theta_Rhos_across_times):
        matched_theta_Rho_i = theta_Rhoi[[np.where(pts_indicei == idx)[0][0] for idx in common_indices]]
        matched_theta_Rho_across_times.append(matched_theta_Rho_i)
    return matched_theta_Rho_across_times, common_indices

theta_Rhos_across_times = []
pts_indices_across_times = []
T_tri_accross_times = []

INITIALIZE_FRAMES = 2 # at least 2 frame
for i in range(INITIALIZE_FRAMES):
    Ti_tri = coordinate_transform_Pose(ros_pose_to_transform_matrix(data[i]['pose']))
    theta_Rhoi = data[i]['si_q_theta_Rho']
    pts_indicei = data[i]['pts_indice']
    T_tri_accross_times.append(Ti_tri)
    theta_Rhos_across_times.append(theta_Rhoi)
    pts_indices_across_times.append(pts_indicei)

    
matched_theta_Rho_across_times, common_indices = get_match_pairs(theta_Rhos_across_times, pts_indices_across_times)
points_num = len(common_indices)
points_num
# Now we have T_tri_accross_times and matched_theta_Rho_across_times

# We need to iterate through points
matched_theta_Rho_across_times = np.array(matched_theta_Rho_across_times)
# for each points
# for point_index in range(points_num):
for point_index in range(1):
    theta_Rhos = matched_theta_Rho_across_times[:,point_index,:]
    
    A_stacked = np.empty((0, 3))
    b_stacked = np.empty((0,))
    A_list = []
    b_list = []

    for i in range(INITIALIZE_FRAMES-1):
        T0 = T_tri_accross_times[i]
        T1 = T_tri_accross_times[i+1]
        T_matrix = np.linalg.inv(T1) @ T0
        theta_Rho = theta_Rhos[i]
        theta_Rho_prime = theta_Rhos[i+1]
        # print(T0, T1)
        # print(theta_Rho, theta_Rho_prime)
        
        
        ##################################################### 
        print("~~~~~~~~~~~~~~~~~~~~~~~~~")
        s_P_cmp, least_square = ANRS(T_matrix, theta_Rho, theta_Rho_prime)
        # Transform back to sim coordinate system
        w_P_cmp = ( T0 @ np.hstack([s_P_cmp, 1]) )[:3]
        # w_P_cmp = coordinate_transform_pt_back( w_P_cmp )
        print("~~~~~~~~~~~~~~~~~~~~~~~~~")
        ##################################################### 
        
        # 将线性方程组写成矩阵形式 A @ P = B
        R_matrix = T_matrix[:3, :3]
        t = T_matrix[:3, 3]
        r1 = R_matrix[0, :]
        r2 = R_matrix[1, :]

        # for theta_Rho, theta_Rho_prime in zip(theta_Rhos, theta_Rho_primes):
        theta = -theta_Rho[0]
        theta_prime = -theta_Rho_prime[0]
        R = theta_Rho[1]  # example value for R
        R_prime = theta_Rho_prime[1] # example value for R'
        
        a1 = np.array([-1, np.tan(theta), 0])
        b1 = 0 
        a2 = np.tan(theta_prime) * r2 - r1
        b2 = t[0] - np.tan(theta_prime) * t[1]
        a3 = t.T @ R_matrix
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2

        A = np.vstack([a1, a2, a3])
        b = np.array([b1, b2, b3])
        # print(A)
        # print(b)

        R0 = T0[:3, :3]
        A = A @ (R0.T)
        # 将 A 和 b 添加到列表中
        A_stacked = np.vstack((A_stacked, A))
        b_stacked = np.append(b_stacked, b)
    # # 循环结束后，将列表转换为 NumPy 数组
    # A_stacked = np.stack(A_list, axis=0)
    # b_stacked = np.stack(b_list, axis=0)


    w_P, residuals, rank, s = np.linalg.lstsq(A_stacked, b_stacked, rcond=None)
    # w_P = coordinate_transform_pt_back( w_P )
    print(w_P, w_P_cmp)


theta_Rho0 = data[0]['si_q_theta_Rho']
pts_indice0 = data[0]['pts_indice']
w_P_gt = data[0]['w_p']
w_P_gt_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
w_P_gt = w_P_gt[w_P_gt_indices] 
print(w_P_gt[0])
