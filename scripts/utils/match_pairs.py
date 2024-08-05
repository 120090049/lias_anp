"""
In real case, we need two consecutive frames to do triangularation 
To reconstruct a 3D points, we need its corresponding projection in two sonar image 

For simplicity, we have index for each points, so we can easily get pairs
"""
import numpy as np

def get_match_pairs(si_q_theta_Rho, pts_indice, si_q_theta_Rho_prime, pts_indice_prime):
    
    # 找到共同的索引
    common_indices = np.intersect1d(pts_indice, pts_indice_prime)

    # 获取t0时刻的匹配坐标
    t0_indices = [np.where(pts_indice == idx)[0][0] for idx in common_indices]
    matched_theta_Rho_t0 = si_q_theta_Rho[t0_indices]

    # 获取t1时刻的匹配坐标
    t1_indices = [np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]
    matched_theta_Rho_t1 = si_q_theta_Rho_prime[t1_indices]
    
    return matched_theta_Rho_t0, matched_theta_Rho_t1, common_indices