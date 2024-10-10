import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

def read_csv_and_plot(file_path):
    # 读取CSV文件
    real_poses = []
    estimated_poses = []
    
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            real_poses.append([float(x) for x in row[:3]])
            estimated_poses.append([float(x) for x in row[3:]])
    
    real_poses = np.array(real_poses)
    estimated_poses = np.array(estimated_poses)
    
    # 分离x, y, z坐标
    real_poses_x, real_poses_y, real_poses_z = real_poses.T
    estimated_poses_x, estimated_poses_y, estimated_poses_z = estimated_poses.T
    
    # 绘制三维轨迹
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    n_points = len(real_poses_x)
    
    # 创建颜色渐变，使用 colormap
    cmap_real = cm.get_cmap('Blues')  # 蓝色渐变 colormap
    colors_real = [cmap_real(i / n_points) for i in range(n_points)]
    
    cmap_estimated = cm.get_cmap('Reds')  # 红色渐变 colormap
    colors_estimated = [cmap_estimated(i / n_points) for i in range(n_points)]
    
    # 绘制真实轨迹，逐段渐变颜色
    for i in range(n_points - 1):
        ax.plot(real_poses_x[i:i+2], real_poses_y[i:i+2], real_poses_z[i:i+2], color=colors_real[i])
    
    # 绘制估计轨迹，逐段渐变颜色
    for i in range(n_points - 1):
        ax.plot(estimated_poses_x[i:i+2], estimated_poses_y[i:i+2], estimated_poses_z[i:i+2], color=colors_estimated[i])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory')
    
    # 添加图例
    ax.plot([], [], 'b-', label='Real Traj')
    ax.plot([], [], 'r-', label='Estimated Traj')
    ax.legend()
    
    ax.grid(True)
    plt.show()

# 使用函数
file_path = "path/to/your/traj.csv"  # 替换为你的CSV文件路径
read_csv_and_plot(file_path)