import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from collections import Counter


class TrajectoryPlotter:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.trajectories = []

    def add_trajectory(self, poses, color, label):
        """
        Add the 3D trajectory of the given poses to the plot.
        
        Parameters:
        poses (list): A list of 4x4 transformation matrices representing the poses.
        color (str)
        label (str): The label for the trajectory.
        """
        self.trajectories.append((poses, color, label))

    def plot_all(self):
        """
        Plot all the added trajectories and display the plot.
        """
        # Plot all the trajectories
        for poses, color, label in self.trajectories:
            plot_color = color[0].lower() + '-'
            # colors = [cm.get_cmap(color + 's')( i / len(poses)) for i in range(len(poses))]
            colors = [cm.get_cmap(color + 's') ( (i + 0.7*len(poses)) / (2*len(poses)) ) for i in range(len(poses))]
            # Extract the x, y, z coordinates from the transformation matrices
            poses_x = [pose[0, 3] for pose in poses]
            poses_y = [pose[1, 3] for pose in poses]
            poses_z = [pose[2, 3] for pose in poses]
            
            n_points = len(poses_x)
            
            # Plot the trajectory with color gradient
            for i in range(n_points - 1):
                self.ax.plot([poses_x[i], poses_x[i+1]], [poses_y[i], poses_y[i+1]], [poses_z[i], poses_z[i+1]], color=colors[i])
            self.ax.plot([], [], plot_color, label=label)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Trajectory')
        
        # Add legend
        self.ax.legend()
        
        self.ax.grid(True)
        plt.show()

def read_csv_file(file_path):
    """
    Read the CSV file and return the real and estimated poses.
    """
    real_poses = []
    estimated_poses = []
    
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            estimated_pose = np.array([float(x) for x in row[:16]]).reshape(4, 4)
            real_pose = np.array([float(x) for x in row[16:]]).reshape(4, 4)
            real_poses.append(real_pose)
            estimated_poses.append(estimated_pose)
    
    return real_poses, estimated_poses


def calculate_ATE(real_poses, estimated_poses):
    """
    Calculate the Absolute Trajectory Error (ATE) between the real and estimated poses.

    Parameters:
    real_poses (list): A list of 4x4 transformation matrices representing the real poses.
    estimated_poses (list): A list of 4x4 transformation matrices representing the estimated poses.

    Returns:
    tuple: (translation_rmse, rotation_rmse)
        translation_rmse (float): The Absolute Trajectory Error (ATE) in translation.
        rotation_rmse (float): The Absolute Trajectory Error (ATE) in rotation.
    """
    assert len(real_poses) == len(estimated_poses), "Real and estimated poses must have the same length."

    translation_errors = []
    rotation_errors = []
    for i in range(len(real_poses)):
        real_pose = real_poses[i]
        estimated_pose = estimated_poses[i]

        # Calculate the transformation error between the real and estimated poses
        pose_error = np.matmul(np.linalg.inv(real_pose), estimated_pose)

        # Extract the translation and rotation components
        translation_component = pose_error[:3, 3]
        rotation_component = pose_error[:3, :3]

        # Calculate the translation error
        translation_error = np.linalg.norm(translation_component)
        translation_errors.append(translation_error)

        # Calculate the rotation error as the sum of squared differences between the elements
        rotation_error = np.sum(np.square(rotation_component - np.eye(3)))
        rotation_errors.append(rotation_error)

    translation_rmse = np.sqrt(np.mean(np.square(translation_errors)))
    rotation_rmse = np.sqrt(np.mean(rotation_errors))

    return translation_rmse, rotation_rmse

def calculate_RTE(real_poses, estimated_poses):
    """
    Calculate the Relative Trajectory Error (RTE) between the real and estimated poses.

    Parameters:
    real_poses (list): A list of 4x4 transformation matrices representing the real poses.
    estimated_poses (list): A list of 4x4 transformation matrices representing the estimated poses.

    Returns:
    tuple: (translation_error, rotation_error)
        translation_error (float): The Relative Trajectory Error (RTE) in translation.
        rotation_error (float): The Relative Trajectory Error (RTE) in rotation.
    """
    assert len(real_poses) == len(estimated_poses), "Real and estimated poses must have the same length."

    translation_errors = []
    rotation_errors = []
    
    for i in range(1, len(real_poses)):
        real_relative_pose = np.linalg.inv(real_poses[i-1]) @ real_poses[i]
        estimated_relative_pose = np.linalg.inv(estimated_poses[i-1]) @ estimated_poses[i]

        # Calculate the transformation error between the real and estimated relative poses
        relative_pose_error = np.matmul(np.linalg.inv(real_relative_pose), estimated_relative_pose)

        # Extract the translation and rotation components
        translation_component = relative_pose_error[:3, 3]
        rotation_component = relative_pose_error[:3, :3]

        # Calculate the translation error
        translation_error = np.linalg.norm(translation_component)
        translation_errors.append(translation_error)

        # Calculate the rotation error as the sum of squared differences between the elements
        rotation_error = np.sum(np.square(rotation_component - np.eye(3)))
        rotation_errors.append(rotation_error)

     
    translation_rmse = np.sqrt(np.mean(np.square(translation_errors)))
    rotation_rmse = np.sqrt(np.mean(rotation_errors))

    return translation_rmse, rotation_rmse

# Usage
file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/square_no3D_gt2D_same_direction/atraj.csv"  # Replace with your CSV file path
real_poses1, estimated_poses_anp = read_csv_file(file_path)
file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/record12/atraj.csv"  # Replace with your CSV file path
real_poses2, estimated_poses_app = read_csv_file(file_path)
file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/circle_no3D_gt2D_same_direction/atraj.csv"  # Replace with your CSV file path
real_poses3, estimated_poses_nonapp = read_csv_file(file_path)

# file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/eight_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses1, estimated_poses_anp = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/app/eight_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses2, estimated_poses_app = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/nonapp/eight_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses3, estimated_poses_nonapp = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/circle_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses1, estimated_poses_anp = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/app/circle_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses2, estimated_poses_app = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/nonapp/circle_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses3, estimated_poses_nonapp = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/anp/square_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses1, estimated_poses_anp = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/app/square_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses2, estimated_poses_app = read_csv_file(file_path)
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/nonapp/square_gt3D_noisy2D/atraj.csv"  # Replace with your CSV file path
# real_poses3, estimated_poses_nonapp = read_csv_file(file_path)
print(calculate_ATE(real_poses1, estimated_poses_anp))
print(calculate_ATE(real_poses2, estimated_poses_app))
print(calculate_ATE(real_poses3, estimated_poses_nonapp))
print(calculate_RTE(real_poses1, estimated_poses_anp))
print(calculate_RTE(real_poses2, estimated_poses_app))
print(calculate_RTE(real_poses3, estimated_poses_nonapp))

# ATE
#       T                       R
# ANP   (0.0009559835213351166, 0.0017780710445350087)
# APP   (0.012842522117435145, 0.017887916294520835)
# NONAPP(0.005813109729101283, 2.014408529318281)

# RTE
#       T                       R
# ANP   (0.001332723699383795, 0.0025714799401775753)
# APP   (0.00530299749664808, 0.008206975704703991)
# NPNAPP(0.06680249098313859, 0.05775331198062487)

length_list = []
for i in range(1, len(real_poses1)):
    translation = real_poses1[i][:3, 3].T - real_poses1[i-1][:3, 3].T
    length_list.append(np.linalg.norm(translation))
length_list.sort(reverse=True)
formatted_list = [float('{:.4g}'.format(num)) for num in length_list]

count = Counter(formatted_list)
for item, frequency in count.items():
    print(f"{item}: {frequency}")


# # 创建直方图
# plt.figure(figsize=(10, 6))
# plt.hist(length_list, bins=100, edgecolor='black')

# # 设置图表标题和轴标签
# plt.title('频率分布直方图')
# plt.xlabel('值')
# plt.ylabel('频率')

# # 显示图表
# plt.show()

plotter = TrajectoryPlotter()

# Add the real trajectory
plotter.add_trajectory(real_poses1, 'Blue', 'Real Traj')
plotter.add_trajectory(real_poses2, 'Blue', 'Real Traj')
plotter.add_trajectory(real_poses3, 'Blue', 'Real Traj')

# Add the estimated trajectory
plotter.add_trajectory(estimated_poses_anp, 'Red', 'anp')
plotter.add_trajectory(estimated_poses_app, 'Green', 'app')
plotter.add_trajectory(estimated_poses_nonapp, 'Purple', 'nonapp')

# Plot all the added trajectories
plotter.plot_all()