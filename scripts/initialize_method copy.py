#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np

def average_lists_numpy(*lists):
    # 将所有列表转换为 NumPy 数组
    arrays = [np.array(l) for l in lists]
    
    # 沿着新轴（axis=0）计算平均值
    return np.mean(arrays, axis=0)


def plot_two_lists(list1, list2, title1="Mean", title2="Variance"):
    # 创建两个子图
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # 绘制第一个列表的折线图
    ax1.plot(list1, marker='o')
    ax1.set_title(title1)
    ax1.set_xlabel('Index')
    ax1.set_ylabel('Value')
    ax1.grid(True)
    
    # 绘制第二个列表的折线图
    ax2.plot(list2, marker='o', color='red')
    ax2.set_title(title2)
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Value')
    ax2.grid(True)
    
    # 调整子图之间的间距
    plt.tight_layout()
    
    # 显示图形
    plt.show()

import matplotlib.pyplot as plt
import sys
import os


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



# INITIALIZE_FRAMES = 2 # at least 2 frame

mean_list = []
variance_list = []

for experiment_times in range(50):
    means = []
    variances = []

    theta_Rhos_across_times = []
    pts_indices_across_times = []
    T_tri_accross_times = []
    for num in range(2, 12):

        INITIALIZE_FRAMES = num
        for i in range(INITIALIZE_FRAMES):
            Ti_tri = coordinate_transform_Pose(ros_pose_to_transform_matrix(data[i]['pose']))
            Ti_tri = add_noise_to_pose(Ti_tri, rotation_noise_std=0.0001, translation_noise_std=0.0001)
            theta_Rhoi = data[i]['si_q_theta_Rho']
            pts_indicei = data[i]['pts_indice']
            T_tri_accross_times.append(Ti_tri)
            theta_Rhos_across_times.append(theta_Rhoi)
            pts_indices_across_times.append(pts_indicei)

            
        matched_theta_Rho_across_times, common_indices = get_match_pairs(theta_Rhos_across_times, pts_indices_across_times)
        points_num = len(common_indices)
        # Now we have T_tri_accross_times and matched_theta_Rho_across_times

        # We need to iterate through points
        matched_theta_Rho_across_times = np.array(matched_theta_Rho_across_times)

        ############################################################
        ## find gt ##
        theta_Rho0 = data[0]['si_q_theta_Rho']
        pts_indice0 = data[0]['pts_indice']
        w_P_gt = data[0]['w_p']
        w_P_gt_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[w_P_gt_indices] 
        ############################################################

        # for each points
        record_ANRS = []
        ##################################################### 
        T0 = T_tri_accross_times[0]
        T1 = T_tri_accross_times[1]
        for point_index in range(points_num):
            theta_Rhos = matched_theta_Rho_across_times[:,point_index,:]
            T_matrix = np.linalg.inv(T1) @ T0
            theta_Rho = theta_Rhos[0]
            theta_Rho_prime = theta_Rhos[1]
            s_P_cmp, least_square = ANRS(T_matrix, theta_Rho, theta_Rho_prime)
            w_P_cmp = ( T0 @ np.hstack([s_P_cmp, 1]) )[:3]
            w_P_cmp = coordinate_transform_pt_back( w_P_cmp )
            difference_ANRS = np.linalg.norm( w_P - w_P_gt[point_index] )
            record_ANRS.append(difference_ANRS)
        mean_ANRS, _ = calculate_mean_variance_numpy(record_ANRS)
        
        record = []
        ##################################################### 
        for point_index in range(points_num):
            theta_Rhos = matched_theta_Rho_across_times[:,point_index,:]
            
            A_stacked = np.empty((0, 4))
            b_stacked = np.empty((0,))
            A_list = []
            b_list = []



            for i in range(INITIALIZE_FRAMES-1):
                T0 = T_tri_accross_times[i]
                T1 = T_tri_accross_times[i+1]
                T_matrix = np.linalg.inv(T1) @ T0
                theta_Rho = theta_Rhos[i]
                theta_Rho_prime = theta_Rhos[i+1]

                
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
                b = np.array([b1, b2, b3, 1])

                H = np.eye(4)
                H[:3, :3] = A
                A = H
                A1 = A @ np.linalg.inv(T0) 

                
                # 将 A 和 b 添加到列表中
                A_stacked = np.vstack((A_stacked, A1))
                b_stacked = np.append(b_stacked, b)
            
            w_P, residuals, rank, s = np.linalg.lstsq(A_stacked, b_stacked, rcond=None)
            w_P = coordinate_transform_pt_back( w_P[:3] )

            difference = np.linalg.norm( w_P - w_P_gt[point_index] )
            record.append(difference)


        def calculate_mean_variance_numpy(numbers):
            arr = np.array(numbers)
            return np.mean(arr), np.var(arr)

        mean, variance = calculate_mean_variance_numpy(record)

        means.append(mean)
        variances.append(variance)
 
    mean_list.append(means)
    variance_list.append(variances)

    plot_two_lists(means, variances)

mean_data = np.mean([np.array(l) for l in mean_list], axis=0)
variance_data = np.mean([np.array(l) for l in variance_list], axis=0)
    


plot_two_lists(mean_data, variance_data)
