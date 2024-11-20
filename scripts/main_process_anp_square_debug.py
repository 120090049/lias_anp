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
from tri.tri import ANRS, GTRS, gradient_descent, reconstrunction_error
from utils.pose2matrix import pose_to_transform_matrix, ros_pose_to_transform_matrix, transform_matrix_to_ros_pose
from utils.coordinate_system_transform import coordinate_transform_Pose, coordinate_transform_Pose_back, coordinate_transform_pt, coordinate_transform_pt_back
from utils.transformation_matrix_add_noise import add_noise_to_pose

import yaml
with open(os.path.join(lias_anp_dir, 'yaml/odom.yaml'), 'r') as file:
    params = yaml.safe_load(file)
    seed = int(params['random_seed'])
    RECONSTRUCTION_ERROR_THRESHOLD = params['RECONSTRUCTION_ERROR_THRESHOLD']
    DETERMINANT_THRESHOLD = params['DETERMINANT_THRESHOLD']
    RECORD = params['RECORD']
    DATA_PATH = params['data_path']
    ANP_METHOD = params['ANP_METHOD']
    rotation_noise_std = float(params['INITIALIZE']['rotation_noise_std'])
    translation_noise_std = float(params['INITIALIZE']['translation_noise_std'])
    
    Rho_noise_std = float(params['SONAR_NOISE']['Rho_noise_std'])
    theta_noise_std = float(params['SONAR_NOISE']['theta_noise_std'])
np.random.seed(seed)

with open(os.path.join(lias_anp_dir, 'yaml/sim_env.yaml'), 'r') as file:
    params = yaml.safe_load(file)
    PHI_MAX = int(params['sonar_attribute']['fov_vertical']) * np.pi / 180

def theta_Rho_add_noise(theta_Rho):
    theta = theta_Rho[:,0]
    Rho = theta_Rho[:,1]
    theta_noise = np.random.normal(0, theta_noise_std, size=theta.shape)
    new_theta = np.arctan(np.tan(theta) + theta_noise)
    Rho_noise = np.random.normal(0, Rho_noise_std, size=Rho.shape) 
    new_Rho = Rho + Rho_noise
    return np.vstack((new_theta, new_Rho)).T 

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
    
    
    
    ###############################################
    ## TRI!! NEED TO TRANSFORM COORDINATE SYSTEM
    ###############################################
    first_index = 0
    second_index = 5 # Need to set properly in order to get a good initialization
    step_size = 3   # Need to set properly in order to get good triangularation  
    if True:
        # Dictionary to store estimated points in world coordinate system
        
        P_dict_0 = {}
        difference_list = []
        reconstruction_error_list = []
        reconstruction_error_list_filtered = []
        # initialize
        T0 = ros_pose_to_transform_matrix(data[first_index]['pose'])
        T1 = ros_pose_to_transform_matrix(data[second_index]['pose']) # This is what we need to initialize
        T0 = add_noise_to_pose(T0, rotation_noise_std=rotation_noise_std, translation_noise_std=translation_noise_std)
        T1 = add_noise_to_pose(T1, rotation_noise_std=rotation_noise_std, translation_noise_std=translation_noise_std)
    
        
        theta_Rho0 = theta_Rho_add_noise(data[first_index]['si_q_theta_Rho'])
        pts_indice0 = data[first_index]['pts_indice']
        
        theta_Rho1 = theta_Rho_add_noise(data[second_index]['si_q_theta_Rho'])
        pts_indice1 = data[second_index]['pts_indice']
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho0, pts_indice0, theta_Rho1, pts_indice1)
        
        # Points ground truth
        w_P_gt = data[first_index]['w_p']
        temp_indices = [np.where(pts_indice0 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[temp_indices] 
        
        T0_tri = coordinate_transform_Pose(T0)
        T1_tri = coordinate_transform_Pose(T1)
        T_matrix = np.linalg.inv(T1_tri) @ T0_tri
        for i in range(len(theta_Rho)):
            s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
            # Transform back to sim coordinate system
            w_P = ( T0_tri @ np.hstack([s_P, 1]) )[:3]
            w_P = coordinate_transform_pt_back( w_P )

            
            theta, R = -theta_Rho[i][0], theta_Rho[i][1]
            theta_prime, R_prime = -theta_Rho_prime[i][0], theta_Rho_prime[i][1]
            ps, ps_prime = np.array([R * np.sin(theta), R * np.cos(theta)]), np.array([R_prime * np.sin(theta_prime), R_prime * np.cos(theta_prime)])
    
            difference = np.linalg.norm( w_P_gt[i] - w_P )
            recon_error = reconstrunction_error(s_P, ps, ps_prime, T_matrix)
            reconstruction_error_list.append(recon_error)
            
            if recon_error < RECONSTRUCTION_ERROR_THRESHOLD and abs(determinant) > DETERMINANT_THRESHOLD :
            
                key = common_indices[i]
                P_dict_0[key] = w_P
                reconstruction_error_list_filtered.append(recon_error)
                difference_list.append(difference)
                
        print(np.mean(np.array(difference_list)), np.var(np.array(difference_list)))
        print(len(reconstruction_error_list), len(reconstruction_error_list_filtered))   
        print(" ".join("{:5.2f}".format(x) for x in difference_list))
        print(" ".join("{:5.2f}".format(x) for x in reconstruction_error_list))

        print()
        
   
    ###############################################
    ## END TRI!! NEED TO TRANSFORM P_W back to original COORDINATE SYSTEM
    ###############################################
    # multi_frame_initialize = int(input("0 for ANRS and 1 for multi_frame: "))
   
    P_dict = P_dict_0
        
    
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
    difference_list = []
    deter_list = []
    reconstruction_error_list = []
    
    # for timestep, entry in enumerate(data[start_index::3], start=start_index):
    input()
    start_index = second_index + step_size
    for timestep in range(start_index, len(data), step_size):
        entry = data[timestep]
        print("==========================================")
        print(f"Timestep: {timestep}") 
        ############################
        ### ANP
        ############################
        ## Get q_si2 and P_w for ANP
        theta_Rho2 = theta_Rho_add_noise(entry['si_q_theta_Rho'])
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
        q_si2 = q_si2.T[filtered_q_si_index]
        P_w = np.array(filtered_P_w_values)
        
        
        w_P_gt = entry['w_p']
        w_P_gt = w_P_gt[filtered_q_si_index] 
        ANP_input_difference_list = []
        for i in range(w_P_gt.shape[0]):
            ANP_input_difference_list.append(np.linalg.norm(w_P_gt[i]-P_w[i]))
        print(" ".join("{:5.2f}".format(x) for x in ANP_input_difference_list))
        print("MAX: ", np.max(ANP_input_difference_list))
        
        print("ANP input size: ", len(q_si2.T))
        print("QSI index", filtered_q_si_index)
        print("-------------------------------------")
        R_SW = ros_pose_to_transform_matrix(entry['pose'])[0:3,0:3]
        t_S = ros_pose_to_transform_matrix(entry['pose'])[0:3,3].reshape(-1,1)
        q_si2 = q_si2.T
        P_w = P_w.T
        R_sw_cal, t_s_cal = anp_algorithm.compute_R_t(P_w, q_si2, phi_max=PHI_MAX, R_true=R_SW)
        # t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(q_si2, P_w, R_true=R_SW, t_true=-R_SW.T@t_S, Var_noise=0)
        T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
        T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
        T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
                
        
        #####################################################
        # TRI
        T_matrix = np.linalg.inv(T2) @ T1
        
        T2_gt = ros_pose_to_transform_matrix(entry['pose'])
        
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho1, pts_indice1, theta_Rho2, pts_indice2)
        w_P_gt = entry['w_p']
        temp_indices = [np.where(pts_indice2 == idx)[0][0] for idx in common_indices]
        w_P_gt = w_P_gt[temp_indices] 
        
    
        determinant_list = []
        
        new_pts_num = 0
        new_pts_valid_num = 0
        
        # This is for tri
        T1_tri = coordinate_transform_Pose(T1)
        T2_tri = coordinate_transform_Pose(T2)
        T_matrix = np.linalg.inv(T2_tri) @ T1_tri

        
        difference_list = []
        deter_list = []
        reconstruction_error_list = []
            
        for i in range(len(theta_Rho)):
            key = common_indices[i]
            if key not in P_dict:
                new_pts_num+=1
                s_P, determinant = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
                determinant_list.append(determinant)

                # Transform back to sim coordinate system
                w_P = ( T1_tri @ np.hstack([s_P, 1]) )[:3]
                w_P = coordinate_transform_pt_back (w_P)
                
                
                theta, R = -theta_Rho[i][0], theta_Rho[i][1]
                theta_prime, R_prime = -theta_Rho_prime[i][0], theta_Rho_prime[i][1]
                ps, ps_prime = np.array([R * np.sin(theta), R * np.cos(theta)]), np.array([R_prime * np.sin(theta_prime), R_prime * np.cos(theta_prime)])
                recon_error = reconstrunction_error(s_P, ps, ps_prime, T_matrix)
                
                difference = np.linalg.norm( w_P_gt[i] - w_P )
                
                # if difference < 0.1  :
                #     P_dict[key] = w_P
                    
                if recon_error < RECONSTRUCTION_ERROR_THRESHOLD and abs(determinant) > DETERMINANT_THRESHOLD :
                    new_pts_valid_num+=1
                    if timestep < 10:
                        key = common_indices[i]
                        P_dict[key] = w_P
                    # if difference > 0.3:
                    #     print()

                    # if difference > 1:
                    #     print()
                    reconstruction_error_list.append(recon_error)
                    difference_list.append(difference)
                    deter_list.append(determinant)
        
        
        print("Dis "+" ".join("{:5.6f}".format(x) for x in difference_list))
        print("Rec "+" ".join("{:5.6f}".format(x) for x in reconstruction_error_list))
        print("Det "+" ".join("{:5.6f}".format(x) for x in deter_list))
            
        theta_Rho1 = theta_Rho2
        pts_indice1 = pts_indice2
        T1 = T2
        
        # cumulative_pose_estimation_error = np.linalg.norm(np.array([real_poses_x, real_poses_y, real_poses_z]) - np.array([estimated_poses_x, estimated_poses_y, estimated_poses_z]))
        print("new_pts_num: ", new_pts_num, "new_pts_valid_num:", new_pts_valid_num)
        if len(difference_list):
            print("max: ", np.max(difference_list), "mean: ", np.mean(difference_list) )
        pose_estimation_error = np.linalg.norm(T2_gt[0:3, 3].reshape(-1) - T2[0:3, 3].reshape(-1))
        # determinant_evaluation = sum(abs(x) for x in determinant_list) / len(determinant_list) if determinant_list else 0
        print(f'\rPose_estimation_error: {pose_estimation_error}')
        if timestep % 20 == 1:
            print()
        
        pre_T = T2_gt     
        
        
        #################################################################
        ## Visualization work
        #################################################################
        T2_gt = ros_pose_to_transform_matrix(entry['pose'])
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
        ax.plot(estimated_poses_x, estimated_poses_y, estimated_poses_z, 'r--', label=ANP_METHOD)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trajectory')
        ax.legend()
        ax.grid(True)

        if RECORD:
            if timestep >= (len(data) - step_size):
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

                ax.set_zlim(-1, 2)
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
            plt.title(ANP_METHOD)
            plt.show(block=False)
            if timestep % 15 == 0:
                plt.pause(1)  # 暂停5秒
            else:
                plt.pause(0.1)
            plt.close()  # 关闭图表窗口
        
