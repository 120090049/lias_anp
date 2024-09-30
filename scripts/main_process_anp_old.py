#!/usr/bin/python3
import numpy as np
import transforms3d

from anp.anp_alg import AnPAlgorithmPython

from tri.tri import ANRS, GTRS, gradient_descent

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import csv
import os


# Append the root dir
import sys, roslib, os
lias_anp_dir = roslib.packages.get_pkg_dir('lias_anp')
scripts_dir = os.path.abspath(os.path.join(lias_anp_dir, 'scripts'))
sys.path.append(scripts_dir)
from utils.sonar_data_processor import SonarDataReader
from matplotlib import cm

RECORD = True

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]

def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵"""
    return transforms3d.quaternions.quat2mat(quaternion)

def pose_to_transform_matrix(pose):
    """将位姿转换为齐次变换矩阵"""
    position = pose['position']
    orientation = pose['orientation']
    # 提取平移向量
    translation = np.array([position['x'], position['y'], position['z']])
    # 提取四元数并转换为旋转矩阵
    quaternion = [orientation['w'], orientation['x'], orientation['y'], orientation['z']]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    
    # 构建齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix

def compute_D(T_matrix, theta, theta_prime):
    """
    Compute the determinant D(A0; R, t) for given parameters.
    """
    R = T_matrix[:3, :3]
    t = T_matrix[:3, 3]
    
    def skew_symmetric_matrix(t):
        """
        Create a skew-symmetric matrix for a vector t.
        """
        return np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
    ux = np.array([1, 0, 0])
    uy = np.array([0, 1, 0])
    
    r1 = R[0, :]
    r2 = R[1, :]
        
    t_cross = skew_symmetric_matrix(t)
    
    determinant = - (r1 - np.tan(theta) * r2).T @ t_cross @ (ux - np.tan(theta_prime) * uy)
    
    return determinant

def get_match_pairs(si_q_theta_Rho, pts_indice, si_q_theta_Rho_prime, pts_indice_prime):
    
    # 找到共同的索引
    common_indices = np.intersect1d(pts_indice, pts_indice_prime)

    # 获取t0时刻的匹配坐标
    t0_indices = [np.where(pts_indice == idx)[0][0] for idx in common_indices]
    matched_t0 = si_q_theta_Rho[t0_indices]

    # 获取t1时刻的匹配坐标
    t1_indices = [np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]
    matched_t1 = si_q_theta_Rho_prime[t1_indices]
    
    return matched_t0, matched_t1, common_indices

def coordinate_transform_T(T0, T1):
    # T1 = T0 @ T
    T_matrix = np.linalg.inv(T0) @ T1 
    # x-axis oriented switched to y-axis oriented
    T_matrix = T_z_90 @ T_matrix @ T_z_min90
    # get transforamtion matrix
    T_matrix = np.linalg.inv(T_matrix)
    return T_matrix

def coordinate_transform_Pose(Pose):
    return (T_z_90 @ Pose @ T_z_min90)

def coordinate_transform_pt(P):
    return (R_z_90 @ P)

def coordinate_transform(p0, p1, T0, T1):
    p0 = coordinate_transform_pt(p0)
    p1 = coordinate_transform_pt(p1)
    T_matrix = coordinate_transform_T(T0, T1)
    return p0, p1, T_matrix



if __name__ == "__main__":

    sonar_data_dir = str(lias_anp_dir) + "/data/naive_eight/sonar_data_noisy.csv"
    record_dir = str(lias_anp_dir) + "/record/anp_old/"
    reader = SonarDataReader(filepath = sonar_data_dir)
    reader.read_data()
    data = reader.get_data()
    
    try:
        file_number = max([int(f[6:]) for f in os.listdir(record_dir) if f.startswith('record') and f[6:].isdigit()])
    except:
        file_number = 0
    record_folder = f"{record_dir}/record{file_number + 1}"
    os.makedirs(record_folder, exist_ok=True)

    anp_algorithm = AnPAlgorithmPython()    
    # initialize
    T0 = pose_to_transform_matrix(data[0]['pose'])
    T1 = pose_to_transform_matrix(data[1]['pose']) # This is what we need to initialize
    
    start_index = 0
    theta_Rho0 = data[start_index]['si_q_theta_Rho']
    pts_indice0 = data[start_index]['pts_indice']
    theta_Rho1 = data[start_index+1]['si_q_theta_Rho']
    pts_indice1 = data[start_index+1]['pts_indice']
    theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho0, pts_indice0, theta_Rho1, pts_indice1)
    
    # Points ground truth
    w_P_gt = data[start_index]['w_p']
    w_P_gt_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
    w_P_gt = w_P_gt[w_P_gt_indices] 
    # w_P_gt = coordinate_transform_pt( w_P_gt.T ).T
    
    # Dictionary to store estimated points in world coordinate system
    P_dict = {}
    reconstruction_error_distance_list = []
    
    ## TRI!! NEED TO TRANSFORM COORDINATE SYSTEM
    T0_tri = coordinate_transform_Pose(T0)
    T1_tri = coordinate_transform_Pose(T1)
    T_matrix = np.linalg.inv(T1_tri) @ T0_tri
    for i in range(len(theta_Rho)):
        # s_P, determinant = GTRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
        s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
        
        # Transform back to sim coordinate system
        w_P = ( T0_tri @ np.hstack([s_P, 1]) )[:3]
        w_P = R_z_90.T @ w_P 
    
        key = common_indices[i]
        P_dict[key] = w_P
        reconstruction_error_distance = np.linalg.norm( w_P_gt[i] - w_P )
        reconstruction_error_distance_list.append(reconstruction_error_distance)
    print(sum(reconstruction_error_distance_list)/len(reconstruction_error_distance_list))   
    ## END TRI!! NEED TO TRANSFORM P_W back to original COORDINATE SYSTEM
    
    # 初始化空列表用于存储轨迹
    real_poses_x = []
    real_poses_y = []
    real_poses_z = []
    estimated_poses_x = []
    estimated_poses_y = []
    estimated_poses_z = []
    
    # General idea is we have T0 and T1, and we want to get T2
    for timestep, entry in enumerate(data[start_index+2:], start=start_index+2):
        print(f"Timestep: {timestep}") 
        # ANP
        ## Get q_si2 and P_w for ANP
        theta_Rho2 = entry['si_q_theta_Rho']
        q_si_x2 = np.cos(theta_Rho2.T[0]) * theta_Rho2.T[1]
        q_si_y2 = np.sin(theta_Rho2.T[0]) * theta_Rho2.T[1]
        q_si2 = np.vstack([q_si_x2, q_si_y2])
        pts_indice2 = entry['pts_indice']

        ##  Find matching pairs of q_si and and w_P in dictionary
        filtered_P_w_values = []
        filtered_q_si_index = []
        for j, idx in enumerate( pts_indice2 ):
            value = P_dict.get(idx)
            if value is not None:
                filtered_P_w_values.append(value[:3])
                filtered_q_si_index.append(j)
        q_si2 = q_si2.T[filtered_q_si_index].T
        P_w = np.array(filtered_P_w_values).T
        
        print("ANP input size: ", len(q_si2.T))
        print("QSI index", filtered_q_si_index)
        t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(q_si2, P_w)
        T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
        T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
        T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
                
        
        #####################################################
        # TRI
        T_matrix = np.linalg.inv(T2) @ T1
        
        T2_gt = pose_to_transform_matrix(entry['pose'])
        
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho1, pts_indice1, theta_Rho2, pts_indice2)
        # Just for double check
        w_P_gt = entry['w_p']
        w_P_gt_indices = [np.where(pts_indice2 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[w_P_gt_indices] 
        reconstrubtion_error_list = []
        
        determinant_list = []
        
        new_pts_num = 0
        new_pts_valid_num = 0
        
        # This is for tri
        T1_tri = coordinate_transform_Pose(T1)
        T2_tri = coordinate_transform_Pose(T2)
        T_matrix = np.linalg.inv(T2_tri) @ T1_tri

        for i in range(len(theta_Rho)):
            # s_P_init, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
            key = common_indices[i]
            if key not in P_dict:
                new_pts_num+=1
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                determinant_list.append(determinant)
                # if abs(determinant) > 0.004:
                if True:
                    
                    # Transform back to sim coordinate system
                    w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
                    w_P = R_z_90.T @ w_P 
                    
                    difference = np.linalg.norm( w_P - w_P_gt[i] )
                    if difference < 0.2:
                        new_pts_valid_num+=1
                        P_dict[key] = w_P
                        reconstrubtion_error_list.append(difference)
                # s_P, good_reconstruct = gradient_descent(s_P_init, theta_Rho[i], theta_Rho_prime[i], T_matrix, tol = 0.01)
                # if good_reconstruct:
            else:
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])

                # Transform back to sim coordinate system
                w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
                w_P = R_z_90.T @ w_P 

                difference = np.linalg.norm( w_P - w_P_gt[i] )
                # ( T1 @ np.hstack([s_P, 1]) )[:3] - w_P_gt[i]
        print("new_pts_num: ", new_pts_num, 
            "new_pts_valid_num:", new_pts_valid_num, 
            "reconstrubtion_error_list maximum error:",\
            max(reconstrubtion_error_list) if reconstrubtion_error_list else "empty")
            
        theta_Rho1 = theta_Rho2
        pts_indice1 = pts_indice2
        
        T1 = T2


        # print(timestep)
        T2_gt = pose_to_transform_matrix(entry['pose'])
        # T2_gt = coordinate_transform_Pose(T2_gt)
        # print()
        # print(T2-T2_gt)
        # 提取x和y坐标（即矩阵的第1和第2列的第4个元素）
        real_poses_x.append(T2_gt[0, 3])
        real_poses_y.append(T2_gt[1, 3])
        real_poses_z.append(T2_gt[2, 3])
        estimated_poses_x.append(T2[0, 3])
        estimated_poses_y.append(T2[1, 3])
        estimated_poses_z.append(T2[2, 3])
    
        
        # 绘制三维轨迹
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(real_poses_x, real_poses_y, real_poses_z, 'b-', label='Real Traj')
        ax.plot(estimated_poses_x, estimated_poses_y, estimated_poses_z, 'r--', label='Estimated Traj')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trajectory')
        ax.legend()
        ax.grid(True)

        pose_estimation_error = np.linalg.norm(np.array([real_poses_x, real_poses_y, real_poses_z]) - np.array([estimated_poses_x, estimated_poses_y, estimated_poses_z]))
        determinant_evaluation = sum(abs(x) for x in determinant_list) / len(determinant_list) if determinant_list else 0
        # reconstrubtion_error_evaluation = sum(abs(x) for x in reconstrubtion_error_list) / len(reconstrubtion_error_list)
        reconstrubtion_error_evaluation = None
        print(f'\rPose_estimation_error: {pose_estimation_error}, reconstrubtion_error_evaluation: {reconstrubtion_error_evaluation}, determinant_evaluation: {determinant_evaluation}')
        if RECORD:
            if timestep == len(data) - 1:
                n_points = len(real_poses_x)
                # 创建颜色渐变，使用 colormap
                cmap = cm.get_cmap('Blues')  # 蓝色渐变 colormap
                colors = [cmap(i / n_points) for i in range(n_points)]  # 生成颜色列表
                # 绘制真实轨迹，逐段渐变颜色

                for i in range(n_points - 1):
                    ax.plot(real_poses_x[i:i+2], real_poses_y[i:i+2], real_poses_z[i:i+2], color=colors[i])

                # 为估计的轨迹使用另一种颜色渐变
                cmap_estimated = cm.get_cmap('Reds')  # 红色渐变 colormap
                colors_estimated = [cmap_estimated(i / n_points) for i in range(n_points)]

                # 绘制估计轨迹，逐段渐变颜色
                for i in range(n_points - 1):
                    ax.plot(estimated_poses_x[i:i+2], estimated_poses_y[i:i+2], estimated_poses_z[i:i+2], color=colors_estimated[i])

                
                plt.show()  # 图像窗口将一直显示，直到你手动关闭它
            
            file_name = f"{record_folder}/time_{timestep}.png"
            plt.savefig(file_name)  # 你可以指定其他文件名和格式，如 'plot.jpg', 'plot.pdf', 等等  
            plt.close()  # 关闭图表窗口
            debug_file = record_folder + "/debug_file.csv"
            with open(debug_file, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestep, pose_estimation_error, reconstrubtion_error_evaluation, determinant_evaluation])
   
               
        else:
            plt.show(block=False)
            if timestep % 15 == 0:
                plt.pause(1)  # 暂停5秒
            else:
                plt.pause(0.1)
            plt.close()  # 关闭图表窗口
        
