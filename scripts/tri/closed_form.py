import numpy as np
import cvxpy as cp

T_z_90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[ 0,0,0,1]])
T_z_min90 = T_z_90.T
R_z_90 = T_z_90[:3, :3]

def compute_D(R, t, theta, theta_prime):
    """
    Compute the determinant D(A0; R, t) for given parameters.
    """
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

def Transformation_matrix(Pose0, Pose1):
    # T1 = T0 @ T
    T_matrix = np.linalg.inv(Pose0) @ Pose1 
    # x-axis oriented switched to y-axis oriented
    T_matrix = T_z_90 @ T_matrix @ T_z_min90
    # get transforamtion matrix
    T_matrix = np.linalg.inv(T_matrix)
    return T_matrix

theta_Rho = np.array([[0.31165045499801636, 3.6955461502075195]]) 
theta_Rho_prime = np.array([[0.08677706122398376, 0.34562981128692627]]) 
Pose0 =  np.array([[0.9831495919305808, -0.18280284430700852, 0.0, -1.6576626320590806], [0.18280284430700852, 0.9831495919305808, 0.0, 0.4605763629956814], [0.0, 0.0, 1.0, 0.2640484375599803], [0.0, 0.0, 0.0, 1.0]])
Pose1 =  np.array([[0.9277975520443236, -0.37308404203417866, 0.0, 1.67979469674189], [0.37308404203417866, 0.9277975520443236, 0.0, -0.09749330968347869], [0.0, 0.0, 1.0, -0.08616022059694262], [0.0, 0.0, 0.0, 1.0]])
P0 = np.array([3.5089612007141113, -1.1304057836532593, -0.25775113701820374, 1])
# P1 = np.array([0.33345890045166016, -0.029009435325860977, 0.08616022020578384])

theta = theta_Rho[0][0]  # 45 degrees
theta_prime = theta_Rho_prime[0][0]  # 30 degrees
R = theta_Rho[0][1]  # example value for R
R_prime = theta_Rho_prime[0][1] # example value for R'


T_matrix = Transformation_matrix(Pose0, Pose1)
R_matrix = T_matrix[:3, :3]
r1 = R_matrix[0, :]
r2 = R_matrix[1, :]
t = T_matrix[:3, 3]


determinant = compute_D(R_matrix, t, theta_Rho[0][0], theta_Rho_prime[0][0])
print("determiant: ", determinant)
##########################################################################

a1 = np.array([-1, np.tan(theta), 0])
b1 = 0 

a2 = np.tan(theta_prime) * r2 - r1
b2 = t[0] - np.tan(theta_prime) * t[1]

a3 = t.T @ R_matrix
b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2

# 将线性方程组写成矩阵形式 A @ P = B
A = np.vstack([a1, a2, a3])
b = np.array([b1, b2, b3])

wanted_result = np.array([3.5089612007141113, -1.1304057836532593, -0.25775113701820374])
# wanted_result = np.array([1.1304057836532593,  -3.5089612007141113, -0.25775113701820374])
print(A @ P0)
print(b)

# P_o = np.linalg.inv(A) @ b
# norm_P_o = np.linalg.norm(P_o)
# P_hat_o = P_o / norm_P_o


# # 構造 \(\tilde{A}\) 和 \(\tilde{b}\)
# A_tilde_prime = np.vstack([A, P_hat_o.T])
# b_tilde_prime = np.append(b, R)

# # 計算 \(\tilde{P}_o'\)
# try:
#     P_tilde_prime = np.linalg.inv(A_tilde_prime.T @ A_tilde_prime) @ A_tilde_prime.T @ b_tilde_prime
#     print("P_tilde_prime:\n", P_tilde_prime)
# except np.linalg.LinAlgError as e:
#     print("Error in computing P_tilde_prime:", e)