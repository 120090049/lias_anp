import csv
import numpy as np

def read_csv_file(file_path):
    """
    Read the CSV file and return the real and estimated poses.
    """
    real_poses = []
    estimated_poses = []
    coordinates_list = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            estimated_pose = np.array([float(x) for x in row[:16]]).reshape(4, 4)
            real_pose = np.array([float(x) for x in row[16:32]]).reshape(4, 4)
            
            # 读取剩余的元素作为坐标点
            coordinates = [float(x) for x in row[32:]]
            # 将坐标点重塑为 (n, 3) 的形状，其中 n 是坐标点的数量
            coordinates = np.array(coordinates).reshape(-1, 3)
            
            real_poses.append(real_pose)
            estimated_poses.append(estimated_pose)
            coordinates_list.append(coordinates)
    
    return real_poses, estimated_poses, coordinates_list