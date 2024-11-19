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
# file_path = "/home/clp/catkin_ws/src/lias_anp/record/ToCAnP/record11/atraj.csv"  # Replace with your CSV file path
file_path = "/home/clp/catkin_ws/src/lias_anp/record/ToCAnP/record17/atraj.csv"  # Replace with your CSV file path
real_poses1, estimated_poses_anp, coordinates_list = read_csv_file(file_path)

print(calculate_ATE(real_poses1, estimated_poses_anp))

print(calculate_RTE(real_poses1, estimated_poses_anp))


length_list = []
for i in range(1, len(real_poses1)):
    translation = real_poses1[i][:3, 3].T - real_poses1[i-1][:3, 3].T
    length_list.append(np.linalg.norm(translation))
length_list.sort(reverse=True)
formatted_list = [float('{:.4g}'.format(num)) for num in length_list]

count = Counter(formatted_list)
for item, frequency in count.items():
    print(f"{item}: {frequency}")


from pose_estimator import PosePredictor
# 反正之后就继续搞，遇到不好的，就cool down 3次
predictor = PosePredictor()
predictor.add_pose(estimated_poses_anp[0])
predictor.add_pose(estimated_poses_anp[1])
predictor.add_pose(estimated_poses_anp[2])
new_trajectory = [estimated_poses_anp[0], estimated_poses_anp[1], estimated_poses_anp[2]]
cool_down = 0
for i, pose in enumerate( estimated_poses_anp[3:-1] ):
    # predicted_pose = predictor.predict_next_pose()
    reasonable, predicted_pose = predictor.is_pose_reasonable(pose)
    if reasonable or cool_down >= 1:
        choosed_pose = pose
        if cool_down >= 1:
            cool_down -= 1
    else:
        choosed_pose = predicted_pose
        cool_down = 3
        
    print(reasonable)
    predictor.add_pose(choosed_pose)
    new_trajectory.append(choosed_pose)
 
# predictor.add_pose(real_poses1[0])
# predictor.add_pose(real_poses1[1])
# predictor.add_pose(real_poses1[2])
# new_trajectory = [real_poses1[0], real_poses1[1], real_poses1[2]]
# for i, pose in enumerate( real_poses1[3:-1] ):
#     # predicted_pose = predictor.predict_next_pose()
#     reasonable, predicted_pose = predictor.is_pose_reasonable(pose)
#     if reasonable:
#         choosed_pose = pose
#     else:
#         choosed_pose = predicted_pose
#     print(reasonable)
#     predictor.add_pose(choosed_pose)
#     new_trajectory.append(choosed_pose)
    # print("Predicted pose:\n", predicted_pose)
    # print("Real pose:\n", real_poses1[i+1])
    # print()

plotter = TrajectoryPlotter()

# Add the real trajectory
plotter.add_trajectory(real_poses1, 'Blue', 'Real Traj')


# Add the estimated trajectory
plotter.add_trajectory(estimated_poses_anp, 'Red', 'anp')
# plotter.add_trajectory(new_trajectory, 'Purple', 'new')

# Plot all the added trajectories
plotter.plot_all()
