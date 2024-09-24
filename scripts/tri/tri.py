#!/usr/bin/python3

import numpy as np
import cvxpy as cp
from scipy.linalg import eig

import sys, roslib, os
project_root = roslib.packages.get_pkg_dir('lias_anp')
root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
sys.path.append(root_dir)

DETERMINANT_THRESHOLD = 0.008
DETERMINANT_THRESHOLD = 0.005

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]

def coordinate_transform_T(Pose0, Pose1):
    # T1 = T0 @ T
    T_matrix = np.linalg.inv(Pose0) @ Pose1 
    # x-axis oriented switched to y-axis oriented
    T_matrix = T_z_90 @ T_matrix @ T_z_min90
    # get transforamtion matrix
    T_matrix = np.linalg.inv(T_matrix)
    return T_matrix

def coordinate_transform_Pose(Pose):
    return (T_z_90 @ Pose)

def coordinate_transform_pt(P):
    return (R_z_90 @ P)

def coordinate_transform(P0, P1, Pose0, Pose1):
    P0 = coordinate_transform_pt(P0)
    P1 = coordinate_transform_pt(P1)
    T_matrix = coordinate_transform_T(Pose0, Pose1)
    return P0, P1, T_matrix

def ANRS(T_matrix, theta_Rho, theta_Rho_prime):
    """_summary_

    Args:
        T_matrix (_type_): _description_
        theta_Rho (_type_): _description_
        theta_Rho_prime (_type_): _description_

    Returns:
        P_tilde_prime, 
        determinant: DETERMINANT_THRESHOLD is recommended to set to 0.005 to filter out estimated point with high error
        _type_: _description_
    """
    # 将线性方程组写成矩阵形式 A @ P = B
    R_matrix = T_matrix[:3, :3]
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]
    t = T_matrix[:3, 3]


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
  

    determinant = np.linalg.det(A)
    # if abs(determinant) > DETERMINANT_THRESHOLD:
    # ANRS
    P_o = np.linalg.inv(A) @ b
    norm_P_o = np.linalg.norm(P_o)
    P_hat_o = P_o / norm_P_o

    A_tilde_prime = np.vstack([A, P_hat_o.T])
    b_tilde_prime = np.append(b, R)
    
    P_tilde_prime = np.linalg.inv(A_tilde_prime.T @ A_tilde_prime) @ A_tilde_prime.T @ b_tilde_prime      
    # else:
    #     P_tilde_prime = None
    return P_tilde_prime, determinant 

def GTRS_old(T_matrix, theta_Rho, theta_Rho_prime):
 
    # Extract components from T_matrix
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > 0.01:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4])

        ATA_be = A.T @ A
        ATb_be = A.T @ b
        D = np.diag([1,1,1,0])
        g = np.zeros(4)
        g[3] = -0.5

        # eig_lambda = eig(D, ATA_be, right=False)
        eig_lambda = np.real(eig(D, ATA_be, right=False))

        lambda_min = -1 / np.max(eig_lambda)

        
        ## 找二分法的初始左端点
        lambda_l = int(lambda_min) 
        y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        if y_l.T @ D @ y_l + 2 * g.T @ y_l < 0:    
            while True:
                # print("Determinant ATA_be", np.linalg.det(ATA_be + lambda_l * D))
                y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
                if y_l.T @ D @ y_l + 2 * g.T @ y_l > 0:
                    break
                lambda_l -= 1
        # already satisfy now we want to refine
        while y_l.T @ D @ y_l + 2 * g.T @ y_l > 0:
            lambda_l += 0.1
            y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        # 细调使其接近0
        while y_l.T @ D @ y_l + 2 * g.T @ y_l <= 0:
            lambda_l -= 0.01
            y_l = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)

        # 找二分法的初始右端点
        lambda_u = lambda_l + 0.1
        
        y_temp = np.linalg.solve(ATA_be + lambda_u * D, ATb_be - lambda_u * g)
        left_check =y_temp.T @ D @ y_temp + 2 * g.T @ y_temp
        y_temp = np.linalg.solve(ATA_be + lambda_l * D, ATb_be - lambda_l * g)
        right_check =y_temp.T @ D @ y_temp + 2 * g.T @ y_temp
        print("GTRS BINARY SEARCH CHECK:", left_check, right_check)
        # 二分法找最优lambda
        while (lambda_u - lambda_l) > 1e-14:
            lambda_temp = (lambda_u + lambda_l) / 2
            y_temp = np.linalg.solve(ATA_be + lambda_temp * D, ATb_be - lambda_temp * g)
            if (y_temp.T @ D @ y_temp + 2 * g.T @ y_temp) > 0:
                lambda_l = lambda_temp
            else:
                lambda_u = lambda_temp
        y = np.linalg.solve(ATA_be + lambda_u * D, ATb_be - lambda_u * g)
        pos = y[:3]
    
    else:
        pos = None
    # distance_constraint = y[3] - np.linalg.norm(pos)**2
    # print('Distance constraint value:', distance_constraint)
    
    return pos, determinant


# CVX
def GTRS_cvx(T_matrix, theta_Rho, theta_Rho_prime):
    print("LALA")
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > 0.01:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4]).reshape(-1, 1)

        ATA = A.T @ A
        ATb = A.T @ b
        
        D = np.diag([1,1,1,0])
        g = np.array([0, 0, 0, -0.5]).reshape(-1, 1)

        # Define variables
        t = cp.Variable(1)
        v = cp.Variable(1)

        # Define the objective
        objective = cp.Minimize(t)
        
        # Define the constraint
        constraint = cp.bmat([
            [ATA + v * D, ATb - v * g],  # First row: (4x4), (4x1)
            [(ATb - v * g).T, cp.reshape(t, (1, 1))]  # Second row: (1x4), (1x1)
        ]) >> 0

        # Solve the problem
        problem = cp.Problem(objective, [constraint])
        problem.solve(solver=cp.SCS, verbose=False)

        # Compute y and x
        y = np.linalg.inv(ATA + v.value * D) @ (ATb - v.value * g)
        x = y[:3]

        # Print the results
    else:
        x = None
    
    return x, determinant

def GTRS(T_matrix, theta_Rho, theta_Rho_prime):

    """     return x, determinant    """
        
    R_matrix = T_matrix[:3, :3]
    t = T_matrix[:3, 3]

    # Extract theta, theta_prime, R, R_prime from inputs
    theta = theta_Rho[0]
    theta_prime = theta_Rho_prime[0]
    R = theta_Rho[1]
    R_prime = theta_Rho_prime[1]

    # Define linear constraints in matrix form A @ P = B
    r1 = R_matrix[0, :]
    r2 = R_matrix[1, :]

    a1 = np.array([-1, np.tan(theta), 0])
    a2 = np.tan(theta_prime) * r2 - r1
    a3 = t.T @ R_matrix
    determinant = np.linalg.det(np.vstack([a1, a2, a3]))
    if abs(determinant) > DETERMINANT_THRESHOLD:
        
        a1 = np.hstack([a1, 0])
        a2 = np.hstack([a2, 0])
        a3 = np.hstack([a3, 0])
        b1 = 0 
        b2 = t[0] - np.tan(theta_prime) * t[1]
        b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2
        a4 = np.array([0, 0, 0, 1])
        b4 = R**2
            
        # 将线性方程组写成矩阵形式 A @ P = B
        A = np.vstack([a1, a2, a3, a4])
        b = np.array([b1, b2, b3, b4]).reshape(-1, 1)

        ATA = A.T @ A
        ATb = A.T @ b
        
        D = np.diag([1,1,1,0])
        f = np.array([0, 0, 0, -0.5]).reshape(-1, 1)
       
        lambda_var = cp.Variable()
        objective = cp.Minimize(lambda_var)
        constraint = [A.T @ A + lambda_var * D >> 0]
        problem = cp.Problem(objective, constraint)
        problem.solve(solver=cp.SCS, verbose=False)
        lambda_val = lambda_var.value
        lambda_l = lambda_val + 2
        lambda_u = lambda_val + 2

        # Define functions for y_l and y_u computation
        def compute_y(A, D, lambda_value, b, f):
            return np.linalg.inv(A.T @ A + lambda_value * D) @ (A.T @ b - lambda_value * f)

        # Compute y_l and adjust lambda_l
        y_l = compute_y(A, D, lambda_l, b, f)
        while (y_l.T @ D @ y_l + 2 * f.T @ y_l).item() < 0:
            lambda_l = (lambda_val + lambda_l) / 2
            y_l = compute_y(A, D, lambda_l, b, f)

        # Compute y_u and adjust lambda_u
        y_u = compute_y(A, D, lambda_u, b, f)
        while (y_u.T @ D @ y_u + 2 * f.T @ y_u).item() > 0:
            lambda_u = lambda_u + (lambda_u - lambda_val) * 2
            y_u = compute_y(A, D, lambda_u, b, f)

        # Perform bisection method to refine lambda
        while (lambda_u - lambda_l) > 1e-14:
            lambda_temp = (lambda_u + lambda_l) / 2
            y_temp = compute_y(A, D, lambda_temp, b, f)
            if (y_temp.T @ D @ y_temp + 2 * f.T @ y_temp).item() > 0:
                lambda_l = lambda_temp
            else:
                lambda_u = lambda_temp

        # Final computation of y and x
        y = compute_y(A, D, lambda_u, b, f)
        x = y[:3]

        # Print the final results
        # print("Final lambda value:", lambda_u)
        # print("Final y:", y)
        # print("Final x:", x)
    else:
        x = None
    
    return x.reshape(-1), determinant

# 定义梯度下降法进行优化
def gradient_descent(P_init, theta_Rho, theta_Rho_prime, T_matrix, learning_rate=0.01, max_iter=1000, tol=1e-3):
    
    def project_to_2d(P):
        X, Y, Z = P
        theta = np.arctan2(X, Y)
        d = np.sqrt(X**2 + Y**2 + Z**2)
        x_s = d * np.sin(theta)
        y_s = d * np.cos(theta)
        return np.array([x_s, y_s])

    # 定义误差函数
    def error_function(P, ps, ps_prime, T_matrix):
        R = T_matrix[:3, :3]
        t = T_matrix[:3, 3]
        # 投影点
        ps_hat = project_to_2d(P)
        P_prime = R @ P + t
        ps_prime_hat = project_to_2d(P_prime)
        
        # 计算误差
        error_ps = ps - ps_hat
        error_ps_prime = ps_prime - ps_prime_hat
        
        # 计算加权误差
        error = error_ps.T @ np.diag(error_ps) @ error_ps + error_ps_prime.T @ np.diag(error_ps_prime) @ error_ps_prime
        return error
    
    theta = theta_Rho[0]
    R = theta_Rho[1]
    theta_prime = theta_Rho_prime[0]
    R_prime = theta_Rho_prime[1]
    ps = np.array([R * np.sin(theta), R * np.cos(theta)])
    ps_prime = np.array([R_prime * np.sin(theta_prime), R_prime * np.cos(theta_prime)])
    
    
    P = P_init
    previous_error = float('inf')
    for i in range(max_iter):
        # 计算当前误差
        error = error_function(P, ps, ps_prime, T_matrix)
        # 计算梯度
        grad = np.zeros_like(P)
        for j in range(len(P)):
            P_temp = P.copy()
            P_temp[j] += tol
            grad[j] = (error_function(P_temp, ps, ps_prime, T_matrix) - error) / tol
        
        # if error increase, we need to take the half of step learning rate
        if previous_error < error:
            learning_rate /= 2
        step = learning_rate * grad
        P_new = P + step
        
        P = P_new
        previous_error = error
        # 检查收敛
        if np.linalg.norm(P_new - P) < tol:
            break
        
        # print(f"Iteration {i+1}, Error: {error}")
    if previous_error > tol:
        return P, False
    return P, True 

if __name__ == "__main__":


    # theta_Rho = np.array([-0.8427304625511169, 2.9157803058624268]) 
    # theta_Rho_prime = np.array([0.10338770598173141, 1.4225620031356812]) 
    # Pose0 =  np.array([[0.9947831471329273, 0.10201220603589002, 0.0, -0.13595595336837749], [-0.10201220603589002, 0.9947831471329273, -0.0, -1.9524370759126866], [-0.0, 0.0, 1.0, -0.3570108264917502], [0.0, 0.0, 0.0, 1.0]])
    # Pose1 =  np.array([[0.8676190607260436, -0.49722948973774467, 0.0, 0.7461405519015483], [0.49722948973774467, 0.8676190607260436, 0.0, -0.5554558458716767], [0.0, 0.0, 1.0, -0.37813731018453933], [0.0, 0.0, 0.0, 1.0]])
    # P0 = np.array([1.92564058303833, 2.1601450443267822, 0.3570108413696289])
    # P1 = np.array([1.3640613555908203, -0.1415318101644516, 0.3781373202800751])

    # P0, P1, T_matrix = coordinate_transform(P0, P1, Pose0, Pose1)

    # Change yaw and z
    
    # Normal
    P0 = np.array([2, 0, 0])
    T_matrix = np.array([
        [0.62866193, -0.77767871, 0.0, -0.20956837],
        [0.77767871, 0.62866193, 0.0, 0.977794],
        [0.0, 0.0, 1.0, -0.62866193],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # 将向量转换为 np.array
    theta_Rho = np.array([0, 2])
    theta_Rho_prime = np.array([-0.66838837, 2.91649294])
    # ANRS_res, deter = ANRS(T_matrix, theta_Rho, theta_Rho_prime)
    # final_res = gradient_descent(ANRS_res, theta_Rho, theta_Rho_prime, T_matrix)
    # print("Calculated P using ANRS:", final_res)
    # print("The Ground truth value: ", P0)


    # # 调用CTOA_GTRS函数
    # GTRS_res, deter = GTRS(T_matrix, theta_Rho, theta_Rho_prime)
    # final_res = gradient_descent(GTRS_res, theta_Rho, theta_Rho_prime, T_matrix)

    # # 显示结果
    # print('Calculated P using GTRS:', GTRS_res)
    # print("The Ground truth value: ", P0)
    
    # print(np.sum(ANRS_res-P0)**2)
    # print(np.sum(GTRS_res-P0)**2)
    s_P_0, determinant0 = ANRS(T_matrix, theta_Rho, theta_Rho_prime)
    s_P_1, determinant1 = GTRS(T_matrix, theta_Rho, theta_Rho_prime)                    
    print("ANRS")
    print(determinant0, s_P_0)
    print("GTRS")
    print(determinant1, s_P_1)
    P0 = np.array([2,0,0])
    
    if s_P_0 is not None:
        print(np.sum(s_P_0-P0)**2)
    if s_P_1 is not None:
        print(np.sum(s_P_1-P0)**2)