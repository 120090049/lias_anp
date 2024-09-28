import numpy as np

def orthogonalize(r1_Noise, r2_Noise):
    angle_Noise_rad = np.arccos(np.dot(r1_Noise, r2_Noise) / (np.linalg.norm(r1_Noise) * np.linalg.norm(r2_Noise)))
    angle_tran = (np.pi / 2 - angle_Noise_rad) / 2
    k = np.cross(r1_Noise, r2_Noise)
    k = k / np.linalg.norm(k)
    r1_Noise_new = (r1_Noise * np.cos(-angle_tran) + 
                    np.cross(k, r1_Noise) * np.sin(-angle_tran) + 
                    k * np.dot(k, r1_Noise) * (1 - np.cos(-angle_tran)))
    r2_Noise_new = (r2_Noise * np.cos(angle_tran) + 
                    np.cross(k, r2_Noise) * np.sin(angle_tran) + 
                    k * np.dot(k, r2_Noise) * (1 - np.cos(angle_tran)))
    return r1_Noise_new, r2_Noise_new

def rot2aa(R):
    theta = np.arccos((np.trace(R) - 1) / 2)
    if theta == 0:
        k = np.array([0, 0, 0])
    else:
        if theta < 0.01:
            print('Warning: theta is too small, bad conditioned problem')
        k = np.array([(R[2, 1] - R[1, 2]),
                      (R[0, 2] - R[2, 0]),
                      (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
    return k, theta

# 生成数据

# 随机生成一个单位正交矩阵
A = np.random.randn(3, 3)
Q, R = np.linalg.qr(A)
Q = Q[:, :3]

print('单位正交矩阵 Q:')
print(Q)
print('验证是否为单位正交矩阵:')
print(np.linalg.norm(Q.T @ Q - np.eye(3)) < np.finfo(float).eps)

R_SW = Q
R_SW = np.array([[-0.5798, 0.4836, -0.6557],
                 [-0.8135, -0.3883, 0.4329],
                 [-0.0453, 0.7844, 0.6186]])
t_S = np.array([6, 4, 7])

num = 6
# P_W = np.random.randn(3, num) * 10

P_W = np.array([[30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15],
                [44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42],
                [35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47]])
# 获取数据的大小
m, n = P_W.shape

# 根据世界坐标生成声纳图像数据
P_S = np.zeros((3, n))
d = np.zeros(n)
cos_theta = np.zeros(n)
sin_theta = np.zeros(n)
tan_theta = np.zeros(n)
theta = np.zeros(n)
cos_phi = np.zeros(n)
P_SI = np.zeros((2, n))

for i in range(n):
    P_S[:, i] = R_SW @ P_W[:, i] + t_S
    d[i] = np.linalg.norm(P_S[:, i])
    cos_theta[i] = P_S[0, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
    sin_theta[i] = P_S[1, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
    tan_theta[i] = sin_theta[i] / cos_theta[i]
    theta[i] = np.arctan(tan_theta[i])
    cos_phi[i] = np.sqrt(P_S[0, i]**2 + P_S[1, i]**2) / d[i]
    P_SI[0, i] = d[i] * cos_theta[i]
    P_SI[1, i] = d[i] * sin_theta[i]

# 数据加噪
Var_Noise = 0.0
P_SI_Noise = P_SI + Var_Noise * np.random.randn(2, n)

d_Noise = np.zeros(n)
cos_theta_Noise = np.zeros(n)
sin_theta_Noise = np.zeros(n)
tan_theta_Noise = np.zeros(n)
theta_N = np.zeros(n)

for i in range(n):
    d_Noise[i] = np.linalg.norm(P_SI_Noise[:, i])
    cos_theta_Noise[i] = P_SI_Noise[0, i] / d_Noise[i]
    sin_theta_Noise[i] = P_SI_Noise[1, i] / d_Noise[i]
    tan_theta_Noise[i] = sin_theta_Noise[i] / cos_theta_Noise[i]
    theta_N[i] = np.arctan(tan_theta_Noise[i])

# 问题求解 加噪 MY
count = 0
Delta_xyz_Noise_my = []
Delta_d_Noise_my = []

for i in range(n):
    for j in range(i + 1, n):
        count += 1
        Delta_xyz_Noise_my.append(2 * (P_W[:, j] - P_W[:, i]))
        Delta_d_Noise_my.append(d_Noise[i]**2 - d_Noise[j]**2 - np.linalg.norm(P_W[:, i])**2 + np.linalg.norm(P_W[:, j])**2)

Delta_xyz_Noise_my = np.array(Delta_xyz_Noise_my)
Delta_d_Noise_my = np.array(Delta_d_Noise_my).reshape(-1, 1)
t_W_Noise_my = np.linalg.inv(Delta_xyz_Noise_my.T @ Delta_xyz_Noise_my) @ Delta_xyz_Noise_my.T @ Delta_d_Noise_my

# 姿态估计 加噪 计算R_SW
A_Noise_my = np.zeros((n, 6))

for i in range(n):
    A_Noise_my[i, 0] = tan_theta_Noise[i] * (P_W[0, i] - t_W_Noise_my[0])
    A_Noise_my[i, 1] = tan_theta_Noise[i] * (P_W[1, i] - t_W_Noise_my[1])
    A_Noise_my[i, 2] = tan_theta_Noise[i] * (P_W[2, i] - t_W_Noise_my[2])
    A_Noise_my[i, 3] = -(P_W[0, i] - t_W_Noise_my[0])
    A_Noise_my[i, 4] = -(P_W[1, i] - t_W_Noise_my[1])
    A_Noise_my[i, 5] = -(P_W[2, i] - t_W_Noise_my[2])

U_Noise_my, S_Noise_my, V_Noise_my = np.linalg.svd(A_Noise_my)
r1_Noise_my = np.sqrt(2) * V_Noise_my.T[:3, 5]
r2_Noise_my = np.sqrt(2) * V_Noise_my.T[3:, 5]

if abs(np.dot(r1_Noise_my, r2_Noise_my)) <= 1e-4:
    print('向量 r1_Noise_my 和向量 r2_Noise_my 是正交的。')
    r3_Noise_my = np.cross(r1_Noise_my, r2_Noise_my)
else:
    print('向量 r1_Noise_my 和向量 r2_Noise_my 不是正交的。')
    r1_Noise_my, r2_Noise_my = orthogonalize(r1_Noise_my, r2_Noise_my)
    if abs(np.dot(r1_Noise_my, r2_Noise_my)) <= 1e-4:
        print('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 是正交的。')
    else:
        print('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 不是正交的。')
    r3_Noise_my = np.cross(r1_Noise_my, r2_Noise_my)
    r1_Noise_my /= np.linalg.norm(r1_Noise_my)
    r2_Noise_my /= np.linalg.norm(r2_Noise_my)
    r3_Noise_my /= np.linalg.norm(r3_Noise_my)

R_Noise_my_1 = np.vstack([r1_Noise_my, r2_Noise_my, r3_Noise_my])
R_Noise_my_2 = np.vstack([r1_Noise_my, r2_Noise_my, -r3_Noise_my])
R_Noise_my_3 = np.vstack([-r1_Noise_my, -r2_Noise_my, r3_Noise_my])
R_Noise_my_4 = np.vstack([-r1_Noise_my, -r2_Noise_my, -r3_Noise_my])

errors = [rot2aa(R_SW.T @ R)[1] for R in [R_Noise_my_1, R_Noise_my_2, R_Noise_my_3, R_Noise_my_4]]
min_error_my = min(errors)
min_error_idx = errors.index(min_error_my)
R_SW_Noise_my = [R_Noise_my_1, R_Noise_my_2, R_Noise_my_3, R_Noise_my_4][min_error_idx]

t_S_Noise_my = -R_SW_Noise_my @ t_W_Noise_my

print(t_S_Noise_my)
print('估计的精度:', rot2aa(R_SW.T @ R_SW_Noise_my))
