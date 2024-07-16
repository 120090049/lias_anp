import numpy as np

def create_translation_matrix(tx, ty, tz):
    T = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])
    return T

def create_rotation_matrix_z(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    Rz = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta, cos_theta, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return Rz

# 平移参数
tx = -2
ty = 0
tz = 0

# 旋转角度（45度转换为弧度）
theta = -np.pi / 4

# 创建平移矩阵和旋转矩阵
translation_matrix = create_translation_matrix(tx, ty, tz)
rotation_matrix_z = create_rotation_matrix_z(theta)

# 组合变换矩阵
combined_matrix = rotation_matrix_z @ translation_matrix

# 打印组合变换矩阵
print("\nCombined Transformation Matrix:")
print(combined_matrix)

# 定义点 P 在坐标系1中的坐标
P1 = np.array([3, 0, 0, 1])

# 计算点 P 在坐标系2中的坐标
P2 = combined_matrix @ P1

# 打印结果
print("\nP 在坐标系1中的坐标: ", P1)
print("P 在坐标系2中的坐标: ", P2)


def create_rotation_matrix_z(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    Rz = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])
    return Rz

R = create_rotation_matrix_z(np.pi / 4)
clp = R.T @ (np.array([3, 0, 0]) - np.array([2, 0, 0]))
print(clp)