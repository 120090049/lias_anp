import numpy as np
import cvxpy as cp

# 已知量（举例值，请根据实际情况替换）
theta = np.pi / 4  # 45 degrees
theta_prime = np.pi / 6  # 30 degrees
R = 4  # example value for R
R_prime = 5  # example value for R'

t = np.array([1, 1, 1])  # example value for t

# 定义旋转矩阵
R_matrix = np.eye(3)  # 示例值，实际需要替换
r1 = R_matrix[0, :]
r2 = R_matrix[1, :]
r3 = R_matrix[2, :]

# 构建 a1, a2, a3 和 b1, b2, b3
a1 = np.array([-1, np.tan(theta), 0, 1])
b1 = 0 + R**2

a2 = np.tan(theta_prime) * r2 - r1
a2 = np.hstack([a2, 1])

b2 = t[0] - np.tan(theta_prime) * t[1] + R**2

a3 = t.T @ R_matrix
a3 = np.hstack([a3, 1])
b3 = (R_prime**2 - R**2 - np.linalg.norm(t)**2) / 2 + R**2

# 将线性方程组写成矩阵形式 A @ P = B
A = np.vstack([a1, a2, a3])
b = np.array([b1, b2, b3])

# AP = b
# 非线性约束条件 ||P|| = ℜ


# 定义问题数据
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
Q_m = A.T @ A 
q_m = (A.T @ b ).reshape(-1,1)

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
print("Optimal value of t:", t_var.value)
print("Optimal value of u:", u_var.value)

# 使用公式 (12) 计算估计值
y_est = np.linalg.inv(Q_m + u_var.value * D) @ (q_m - u_var.value * v)
print("Estimated y:", y_est)
