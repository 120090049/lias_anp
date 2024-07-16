import numpy as np
import cvxpy as cp


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
# D: 0.026109026510045763, 

# R: [[0.9990536264091424, 0.04349541997431439, 0.0], 
# [-0.04349541997431439, 0.9990536264091424, 0.0], 
# [0.0, 0.0, 1.0]], 

# t: [0.043012375553797444, -0.05713717158598475, -0.15096811108291142], 
# theta_Rho: [[0.7963768839836121, 3.930614709854126]], 
# theta_Rho_prime: [[0.8342011570930481, 3.9380810260772705]]

# {
# 'Pose': [[0.9831495919305808, -0.18280284430700852, 0.0, -1.6576626320590806], [0.18280284430700852, 0.9831495919305808, 0.0, 0.4605763629956814], [0.0, 0.0, 1.0, 0.2640484375599803], [0.0, 0.0, 0.0, 1.0]], 
# 'theta_Rho': [[0.31165045499801636, 3.6955461502075195]], 
# 's_p': [[3.5089612007141113, -1.1304057836532593, -0.25775113701820374]], 
# 'w_p': [[2.0, 0.0, 0.0]]
# }

# {
# 'Pose': [[0.9277975520443236, -0.37308404203417866, 0.0, 1.67979469674189], [0.37308404203417866, 0.9277975520443236, 0.0, -0.09749330968347869], [0.0, 0.0, 1.0, -0.08616022059694262], [0.0, 0.0, 0.0, 1.0]], 
# 'theta_Rho': [[0.08677706122398376, 0.34562981128692627]], 
# 's_p': [[0.33345890045166016, -0.029009435325860977, 0.08616022020578384]], 
# 'w_p': [[2.0, 0.0, 0.0]]
# }

theta_Rho = np.array([[0.31165045499801636, 3.6955461502075195]]) 
theta_Rho_prime = np.array([[0.08677706122398376, 0.34562981128692627]]) 
# 定义旋转矩阵
Pose0 =  np.array([[0.9831495919305808, -0.18280284430700852, 0.0, -1.6576626320590806], [0.18280284430700852, 0.9831495919305808, 0.0, 0.4605763629956814], [0.0, 0.0, 1.0, 0.2640484375599803], [0.0, 0.0, 0.0, 1.0]])
Pose1 =  np.array([[0.9277975520443236, -0.37308404203417866, 0.0, 1.67979469674189], [0.37308404203417866, 0.9277975520443236, 0.0, -0.09749330968347869], [0.0, 0.0, 1.0, -0.08616022059694262], [0.0, 0.0, 0.0, 1.0]])


theta = theta_Rho[0][0]  # 45 degrees
theta_prime = theta_Rho_prime[0][0]  # 30 degrees
R = theta_Rho[0][1]  # example value for R
R_prime = theta_Rho_prime[0][1] # example value for R'


T_matrix = Pose1 @ np.linalg.inv(Pose0)

R_matrix = T_matrix[:3, :3]
t = T_matrix[:3, 3]
r1 = R_matrix[0, :]
r2 = R_matrix[1, :]


determinant = compute_D(R_matrix, t, theta_Rho[0][0], theta_Rho_prime[0][0])
print("determiant: ", determinant)
##########################################################################

a1 = np.array([-1, np.tan(theta), 0, 0])
b1 = 0 

a2 = np.tan(theta_prime) * r2 - r1
a2 = np.hstack([a2, 0])
b2 = t[0] - np.tan(theta_prime) * t[1]

a3 = t.T @ R_matrix
a3 = np.hstack([a3, 0])
b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2

a4 = np.array([0, 0, 0, 1])
b4 = R**2

# 将线性方程组写成矩阵形式 A @ P = B
A = np.vstack([a1, a2, a3, a4])
b = np.array([b1, b2, b3, b4])

n = 3   # 变量维度

# 定义D和v
D = np.block([
    [np.eye(n), np.zeros((n, 1))],
    [np.zeros((1, n)), 0]
])
v = np.block([
    [np.zeros((n, 1))],
    [-0.5]
])

# 定义优化变量
t_var = cp.Variable()
u_var = cp.Variable()

# 定义矩阵Q_m和q_m
Q_m = A.T @ A / 2
q_m = (A.T @ b / 2).reshape(-1,1)

# 构建半定规划问题的约束
constraints = [
    cp.bmat([
        [Q_m + u_var * D, q_m - u_var * v],
        [(q_m - u_var * v).T, cp.reshape(t_var, (1, 1))]
    ]) >> 0
]

# 定义目标函数
objective = cp.Minimize(t_var)

# 求解优化问题
prob = cp.Problem(objective, constraints)
result = prob.solve()

# 打印结果
# print("Optimal value of t:", t_var.value)
# print("Optimal value of u:", u_var.value)

# 使用公式 (12) 计算估计值
y_est = np.linalg.inv(Q_m + u_var.value * D) @ (q_m - u_var.value * v)
print("Estimated y:", y_est)
print(np.sum(y_est[:3] ** 2))

################################################################