#!/usr/bin/python3
import numpy as np

import matplotlib.pyplot as plt
import sys
import csv
import os
from mpl_toolkits.mplot3d import Axes3D

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

if __name__ == "__main__":

    sonar_data_dir = str(lias_anp_dir) + DATA_PATH
    reord_dir = str(lias_anp_dir) + "/record/" + ANP_METHOD
    reader = SonarDataReader(filepath = sonar_data_dir)
    reader.read_data()
    data = reader.get_data()
    
    try:
        file_number = max([int(f[6:]) for f in os.listdir(reord_dir) if f.startswith('record') and f[6:].isdigit()])
    except:
        file_number = 0
    record_folder = f"{reord_dir}/record{file_number + 1}"
    os.makedirs(record_folder, exist_ok=True)

    anp_algorithm = AnPAlgorithm(ANP_METHOD)
    
    
    # Dictionary to store estimated points in world coordinate system
    P_dict = {}
    reconstruction_error_list = []
    reconstruction_error_list_filtered = []
    
    ###############################################
    ## TRI!! NEED TO TRANSFORM COORDINATE SYSTEM
    ###############################################
    ANRS_initialize = True
    if True:
        # initialize
        T0 = ros_pose_to_transform_matrix(data[0]['pose'])
        T1 = ros_pose_to_transform_matrix(data[1]['pose']) # This is what we need to initialize
        T0 = add_noise_to_pose(T0, rotation_noise_std=0.00001, translation_noise_std=0.0001)
        T1 = add_noise_to_pose(T1, rotation_noise_std=0.00001, translation_noise_std=0.0001)
        
        theta_Rho0 = data[0]['si_q_theta_Rho']
        pts_indice0 = data[0]['pts_indice']
        theta_Rho1 = data[1]['si_q_theta_Rho']
        pts_indice1 = data[1]['pts_indice']
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho0, pts_indice0, theta_Rho1, pts_indice1)
        
        # Points ground truth
        w_P_gt = data[0]['w_p']
        w_P_gt_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[w_P_gt_indices] 
        
        T0_tri = coordinate_transform_Pose(T0)
        T1_tri = coordinate_transform_Pose(T1)
        T_matrix = np.linalg.inv(T1_tri) @ T0_tri
        for i in range(len(theta_Rho)):
            s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
            # Transform back to sim coordinate system
            w_P = ( T0_tri @ np.hstack([s_P, 1]) )[:3]
            w_P = coordinate_transform_pt_back( w_P )

            key = common_indices[i]
            difference = np.linalg.norm( w_P_gt[i] - w_P )
            reconstruction_error_list.append(difference)
            
            if difference < RECONSTRUCTION_ERROR_THRESHOLD:
                if ANRS_initialize:
                    P_dict[key] = w_P
                reconstruction_error_list_filtered.append(difference)
                
        print(np.mean(np.array(reconstruction_error_list)), np.var(np.array(reconstruction_error_list)))
        print(len(reconstruction_error_list), len(reconstruction_error_list_filtered))   
        print()
    if True:
        ###############################################
        INITIALIZE_FRAMES = 5
        step_size = 1
        
        # Add some noise to the poses
        T_tri_accross_times = []
        for i in range(INITIALIZE_FRAMES):
            Ti_tri = coordinate_transform_Pose(ros_pose_to_transform_matrix(data[step_size*i]['pose']))
            Ti_tri = add_noise_to_pose(Ti_tri, rotation_noise_std=0.0001, translation_noise_std=0.0005)
            T_tri_accross_times.append(Ti_tri)
        
        # read data
        theta_Rhos_across_times = []
        pts_indices_across_times = []
        for i in range(INITIALIZE_FRAMES):
            theta_Rhoi = data[step_size*i]['si_q_theta_Rho']
            pts_indicei = data[step_size*i]['pts_indice']
            theta_Rhos_across_times.append(theta_Rhoi)
            pts_indices_across_times.append(pts_indicei)
    
        matched_theta_Rho_across_times, common_indices = get_match_pairs(theta_Rhos_across_times, pts_indices_across_times)
        matched_theta_Rho_across_times = np.array(matched_theta_Rho_across_times)
        points_num = len(common_indices)
    
        ## find gt ##
        w_P_gt = data[0]['w_p']
        pts_indice0 = data[0]['pts_indice']
        w_P_gt_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[w_P_gt_indices] 
        
        # Now we have T_tri_accross_times and matched_theta_Rho_across_times
        # We need to iterate through points
    
        record = []
        record_filtered = []
        ##################################################### 
        # for each points
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
            
            if difference < RECONSTRUCTION_ERROR_THRESHOLD:
                if not ANRS_initialize:
                    P_dict[key] = w_P
                record_filtered.append(difference)
                    
                    
            record.append(difference)
        print(np.mean(np.array(record)), np.var(np.array(record)))
        print(len(record), len(record_filtered))

    ###############################################
    ## END TRI!! NEED TO TRANSFORM P_W back to original COORDINATE SYSTEM
    ###############################################
        
    
    # 初始化空列表用于存储轨迹
    real_poses_x = []
    real_poses_y = []
    real_poses_z = []
    estimated_poses_x = []
    estimated_poses_y = []
    estimated_poses_z = []
    
    ##############################################################
    ## General idea is we have T0 and T1, and we want to get T2 ##
    ##############################################################
    if ANRS_initialize:
        start_index = 2
    else:    
        start_index = INITIALIZE_FRAMES
    for timestep, entry in enumerate(data[start_index:], start=start_index):
        print(f"Timestep: {timestep}") 
        ############################
        ### ANP
        ############################
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
        
        R_SW = ros_pose_to_transform_matrix(entry['pose'])[0:3,0:3]
        t_S = ros_pose_to_transform_matrix(entry['pose'])[0:3,3].reshape(-1,1)
        t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(q_si2, P_w, R_true=R_SW, t_true=-R_SW.T@t_S, Var_noise=0)
        T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
        T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
        T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
                
        
        #####################################################
        # TRI
        T_matrix = np.linalg.inv(T2) @ T1
        
        T2_gt = ros_pose_to_transform_matrix(entry['pose'])
        
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho1, pts_indice1, theta_Rho2, pts_indice2)
        w_P_gt = entry['w_p']
        w_P_gt_indices = [np.where(pts_indice2 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[w_P_gt_indices] 
        reconstruction_error_list = []
        
        determinant_list = []
        
        new_pts_num = 0
        new_pts_valid_num = 0
        
        # This is for tri
        T1_tri = coordinate_transform_Pose(T1)
        T2_tri = coordinate_transform_Pose(T2)
        T_matrix = np.linalg.inv(T2_tri) @ T1_tri

        for i in range(len(theta_Rho)):
            key = common_indices[i]
            if key not in P_dict:
                new_pts_num+=1
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                determinant_list.append(determinant)
                if True:
                    # Transform back to sim coordinate system
                    w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
                    w_P = coordinate_transform_pt_back (w_P)
                    
                    difference = np.linalg.norm( w_P - w_P_gt[i] )
                    if difference < RECONSTRUCTION_ERROR_THRESHOLD:
                        new_pts_valid_num+=1
                        P_dict[key] = w_P
                        reconstruction_error_list.append(difference)
                # s_P, good_reconstruct = gradient_descent(s_P_init, theta_Rho[i], theta_Rho_prime[i], T_matrix, tol = 0.01)
                # if good_reconstruct:
            else:
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                # Transform back to sim coordinate system
                w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
                w_P = coordinate_transform_pt_back (w_P)
                difference = np.linalg.norm( w_P - w_P_gt[i] )

        print("new_pts_num: ", new_pts_num, 
            "new_pts_valid_num:", new_pts_valid_num, 
            "reconstruction_error_list maximum error:",\
            max(reconstruction_error_list) if reconstruction_error_list else "empty")
            
        theta_Rho1 = theta_Rho2
        pts_indice1 = pts_indice2
        
        T1 = T2


        # print(timestep)
        T2_gt = ros_pose_to_transform_matrix(entry['pose'])
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

        # cumulative_pose_estimation_error = np.linalg.norm(np.array([real_poses_x, real_poses_y, real_poses_z]) - np.array([estimated_poses_x, estimated_poses_y, estimated_poses_z]))
        pose_estimation_error = np.linalg.norm(T2_gt[0:3, 3].reshape(-1) - T2[0:3, 3].reshape(-1))
        determinant_evaluation = sum(abs(x) for x in determinant_list) / len(determinant_list) if determinant_list else 0
        # reconstrubtion_error_evaluation = sum(abs(x) for x in reconstruction_error_list) / len(reconstruction_error_list)
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
            debug_file = record_folder + "/atraj.csv"
            values_array = np.array(list(P_dict.values()))
            with open(debug_file, 'a', newline='') as file:
                writer = csv.writer(file)
                row = np.concatenate([T2.flatten(), T2_gt.flatten(), values_array.flatten()])
                writer.writerow(row)
               
        else:
            plt.show(block=False)
            if timestep % 15 == 0:
                plt.pause(1)  # 暂停5秒
            else:
                plt.pause(0.1)
            plt.close()  # 关闭图表窗口
        
