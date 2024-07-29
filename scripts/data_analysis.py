import matplotlib.pyplot as plt
import pandas as pd

# 读取CSV文件
df = pd.read_csv('debug_file_GTRS.csv', header=None, names=['timestep', 'pose_estimation_error', 'avg_determinant', 'avg_pts_reconstruct'])
df = df[:35]
# 确保所有列都是数值类型
df['timestep'] = pd.to_numeric(df['timestep'])
df['pose_estimation_error'] = pd.to_numeric(df['pose_estimation_error'])
df['avg_determinant'] = pd.to_numeric(df['avg_determinant'])
df['avg_pts_reconstruct'] = pd.to_numeric(df['avg_pts_reconstruct'])

# 绘制折线图
plt.figure(figsize=(10, 6))

# 绘制每一列的数据
plt.plot(df['timestep'].values, 3*df['pose_estimation_error'].values, label='pose_estimation_error')
plt.plot(df['timestep'].values, 100*df['avg_determinant'].values, label='avg_determinant')
plt.plot(df['timestep'].values, 3*df['avg_pts_reconstruct'].values, label='avg_pts_reconstruct')

# 添加标题和标签
plt.title('Data over Time')
plt.xlabel('Timestep')
plt.ylabel('Values')
plt.legend()

# 显示图表
plt.show()
