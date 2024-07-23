from sonardatareader import SonarDataReader
import numpy as np
import transforms3d
from anp.anp import AnPAlgorithm
from tri.tri import ANRS, GTRS

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

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
    reader = SonarDataReader(filepath = "sonar_data.csv")
    reader.read_data()
    data = reader.get_data()
    
    anp_algorithm = AnPAlgorithm()
    
    # initialize
    T0 = pose_to_transform_matrix(data[0]['pose'])
    T0 = coordinate_transform_Pose(T0)
    T1 = pose_to_transform_matrix(data[1]['pose']) # This is what we need to initialize
    T1 = coordinate_transform_Pose(T1)
    T_matrix = np.linalg.inv(T1) @ T0

    theta_Rho0 = data[0]['si_q_theta_Rho']
    pts_indice0 = data[0]['pts_indice']
    theta_Rho1 = data[1]['si_q_theta_Rho']
    pts_indice1 = data[1]['pts_indice']
    
    # Dictionary to store estimated points in world coordinate system
    P_dict = {}
    theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho0, pts_indice0, theta_Rho1, pts_indice1)
    for i in range(len(theta_Rho)):
        # determinant = compute_D(T_matrix, theta=theta_Rho[timestep][0], theta_prime=theta_Rho_prime[timestep][0])
        s_P = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
        w_P = ( T0 @ np.hstack([s_P, 1]) )[:3]
        key = common_indices[i]
        P_dict[key] = w_P
    
    # 初始化空列表用于存储轨迹
    real_poses_x = []
    real_poses_y = []
    real_poses_z = []
    estimated_poses_x = []
    estimated_poses_y = []
    estimated_poses_z = []
    # General idea is we have T0 and T1, and we want to get T2
    for timestep, entry in enumerate(data[2:], start=2):
        sys.stdout.write(f'\rTimestep: {timestep}')
        sys.stdout.flush()
        # ANP
        ## Get q_si2 and P_w for ANP
        theta_Rho2 = entry['si_q_theta_Rho']
        q_si_x2 = np.sin(theta_Rho2.T[0]) * theta_Rho2.T[1]
        q_si_y2 = np.cos(theta_Rho2.T[0]) * theta_Rho2.T[1]
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
        t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(q_si2, P_w)
        T2 = np.eye(4)  # 创建一个 4x4 的单位矩阵
        T2[:3, :3] = R_sw_cal  # 将 R 赋值给 T 的左上 3x3 子矩阵
        T2[:3, 3] = t_s_cal.flatten()  # 将 t 赋值给 T 的前 3 行第 4 列
        T2 = np.linalg.inv(T2)
        
        #####################################################
        # TRI
        T_matrix = np.linalg.inv(T2) @ T1
        
        # theta_Rho2 = entry['si_q_theta_Rho']
        # pts_indice2 = entry['pts_indice']
        
        theta_Rho, theta_Rho_prime, common_indices = get_match_pairs(theta_Rho1, pts_indice1, theta_Rho2, pts_indice2)

        for i in range(len(theta_Rho)):
            # determinant = compute_D(T_matrix, theta=theta_Rho[timestep][0], theta_prime=theta_Rho_prime[timestep][0])
            s_P = ANRS(T_matrix, theta_Rho[i], theta_Rho_prime[i])
            w_P = ( T1 @ np.hstack([s_P, 1]) )[:3]
            key = common_indices[i]
            P_dict[key] = w_P
        
        theta_Rho1 = theta_Rho2
        pts_indice1 = pts_indice2
        # print(P_dict)
        
        T1 = T2


        # print(timestep)
        T2_gt = pose_to_transform_matrix(entry['pose'])
        T2_gt = coordinate_transform_Pose(T2_gt)
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

        plt.show()