import numpy as np
import matlab.engine
from pathlib import Path

# 获取脚本的路径
script_path = Path(__file__).resolve()
# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"

class AnPAlgorithmPython:
    def __init__(self):
        # t_s, R_sw
        self.R_sw = None
        self.t_s = None

    @staticmethod
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

    def compute_t_R(self, P_SI, P_W):
        num = P_SI.shape[1]
        d_Noise = np.zeros(num)
        cos_theta_Noise = np.zeros(num)
        sin_theta_Noise = np.zeros(num)
        tan_theta_Noise = np.zeros(num)
        theta_N = np.zeros(num)

        for i in range(num):
            d_Noise[i] = np.linalg.norm(P_SI[:, i])
            cos_theta_Noise[i] = P_SI[0, i] / d_Noise[i]
            sin_theta_Noise[i] = P_SI[1, i] / d_Noise[i]
            tan_theta_Noise[i] = sin_theta_Noise[i] / cos_theta_Noise[i]
            theta_N[i] = np.arctan(tan_theta_Noise[i])

        count = 0
        Delta_xyz_Noise_my = []
        Delta_d_Noise_my = []

        for i in range(num):
            for j in range(i + 1, num):
                count += 1
                Delta_xyz_Noise_my.append(2 * (P_W[:, j] - P_W[:, i]))
                Delta_d_Noise_my.append(d_Noise[i]**2 - d_Noise[j]**2 - np.linalg.norm(P_W[:, i])**2 + np.linalg.norm(P_W[:, j])**2)

        Delta_xyz_Noise_my = np.array(Delta_xyz_Noise_my)
        Delta_d_Noise_my = np.array(Delta_d_Noise_my).reshape(-1, 1)
        t_W_Noise_my = np.linalg.inv(Delta_xyz_Noise_my.T @ Delta_xyz_Noise_my) @ Delta_xyz_Noise_my.T @ Delta_d_Noise_my

        A_Noise_my = np.zeros((num, 6))

        for i in range(num):
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
            # print('向量 r1_Noise_my 和向量 r2_Noise_my 是正交的。')
            r3_Noise_my = np.cross(r1_Noise_my, r2_Noise_my)
        else:
            # print('向量 r1_Noise_my 和向量 r2_Noise_my 不是正交的。')
            r1_Noise_my, r2_Noise_my = self.orthogonalize(r1_Noise_my, r2_Noise_my)
            # if abs(np.dot(r1_Noise_my, r2_Noise_my)) <= 1e-4:
            #     print('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 是正交的。')
            # else:
            #     print('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 不是正交的。')
            r3_Noise_my = np.cross(r1_Noise_my, r2_Noise_my)
            r1_Noise_my /= np.linalg.norm(r1_Noise_my)
            r2_Noise_my /= np.linalg.norm(r2_Noise_my)
            r3_Noise_my /= np.linalg.norm(r3_Noise_my)

        R_Noise_my_1 = np.vstack([r1_Noise_my, r2_Noise_my, r3_Noise_my])
        R_Noise_my_2 = np.vstack([r1_Noise_my, r2_Noise_my, -r3_Noise_my])
        R_Noise_my_3 = np.vstack([-r1_Noise_my, -r2_Noise_my, r3_Noise_my])
        R_Noise_my_4 = np.vstack([-r1_Noise_my, -r2_Noise_my, -r3_Noise_my])
        
        
        # 根据 R_sw 估计声呐坐标系中的坐标 P_S
        P_S_Estimate_my_1 = R_Noise_my_1 @ (P_W - t_W_Noise_my)
        P_S_Estimate_my_2 = R_Noise_my_2 @ (P_W - t_W_Noise_my)
        P_S_Estimate_my_3 = R_Noise_my_3 @ (P_W - t_W_Noise_my)
        P_S_Estimate_my_4 = R_Noise_my_4 @ (P_W - t_W_Noise_my)

        # 计算估计的 cos(theta)
        cos_theta_vatify_1 = P_S_Estimate_my_1[0, 0] / np.sqrt(P_S_Estimate_my_1[0, 0]**2 + P_S_Estimate_my_1[1, 0]**2)
        cos_theta_vatify_2 = P_S_Estimate_my_2[0, 0] / np.sqrt(P_S_Estimate_my_2[0, 0]**2 + P_S_Estimate_my_2[1, 0]**2)
        cos_theta_vatify_3 = P_S_Estimate_my_3[0, 0] / np.sqrt(P_S_Estimate_my_3[0, 0]**2 + P_S_Estimate_my_3[1, 0]**2)
        cos_theta_vatify_4 = P_S_Estimate_my_4[0, 0] / np.sqrt(P_S_Estimate_my_4[0, 0]**2 + P_S_Estimate_my_4[1, 0]**2)

        # 计算真值 cos(theta)
        cos_theta_true = P_SI[0, 0] / np.sqrt(P_SI[0, 0]**2 + P_SI[1, 0]**2)

        # 选择最优的 R_sw
        if cos_theta_vatify_1 * cos_theta_true > 0:
            R_sw = R_Noise_my_1
        elif cos_theta_vatify_2 * cos_theta_true > 0:
            R_sw = R_Noise_my_2
        elif cos_theta_vatify_3 * cos_theta_true > 0:
            R_sw = R_Noise_my_3
        elif cos_theta_vatify_4 * cos_theta_true > 0:
            R_sw = R_Noise_my_4
        else:
            raise ValueError("No valid R_sw found")

        self.t_s = t_W_Noise_my
        
        T = np.eye(4)
        T[:3, :3] = R_sw  # 将旋转矩阵 R 放入左上角 3x3 部分
        T[:3, 3] = t_W_Noise_my.reshape(-1)
        T = np.linalg.inv(T) 
        R_sw = T[:3, :3] 
        self.R_sw = R_sw
        
        return self.t_s, self.R_sw

    @staticmethod
    def rot2aa(R):
        """
        Converts a 3x3 rotation matrix to axis-angle representation.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing a rotation in 3D space.

        Returns:
            tuple: A tuple containing:
                - k (numpy.ndarray): A 3D unit vector representing the axis of rotation. If no rotation is present (theta == 0), k is [0, 0, 0].
                - theta (float): The rotation angle in radians, representing the amount of rotation around the axis k.
        """
        # 计算旋转角度 theta
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta == 0: # 如果 theta 为零，意味着没有旋转，此时旋转轴向量 k 无意义
            k = np.array([0, 0, 0])
        else: # 如果 theta 不为零，使用以下公式计算旋转轴 k
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta

    def estimate_accuracy(self, R_sw_gt):
        """
        Evaluate the accuracy of the estimated rotation matrix by comparing it to the ground truth.

        This function computes the axis-angle difference between the estimated rotation matrix and 
        the ground truth rotation matrix. It returns the axis of rotation and the angular deviation (the closer to 0, the better).

        Args:
            R_sw_gt (numpy.ndarray): The ground truth rotation matrix, which is a 3x3 numpy array 
                                    representing the true rotation.

        Returns:
            tuple: A tuple containing:
                - axis (numpy.ndarray): A 3D unit vector representing the axis of rotation difference.
                - theta (float): The angular deviation between the estimated and ground truth rotation 
                                matrices, in radians.
        """
        axis, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return axis, theta


class AnPAlgorithmMatlab:
    def __init__(self):
        # t_s, R_sw
        self.R_sw = None
        self.t_s = None
        # 启动 MATLAB 引擎
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(script_dir)
        # self.eng.addpath('/home/clp/catkin_ws/src/lias_anp/scripts/anp/')
        print("Start matlab engine, ANP module successfully initialized!")
        
    def compute_t_R(self, P_SI, P_W):
        # 将 numpy 数组转换为 matlab.double 类型
        P_SI_matlab = matlab.double(P_SI.tolist())
        P_W_matlab = matlab.double(P_W.tolist())

        # 调用 MATLAB 中的 compute_t_R 函数
        R_sw, t_s = self.eng.compute_t_R(P_W_matlab, P_SI_matlab, 0, 0, nargout=2)

        # 将结果保存到类的属性中
        self.R_sw = np.array(R_sw).T
        self.t_s = np.array(t_s)
        # self.t_s = -self.R_sw @ t_s

        return self.t_s, self.R_sw

    @staticmethod
    def rot2aa(R):
        """
        Converts a 3x3 rotation matrix to axis-angle representation.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing a rotation in 3D space.

        Returns:
            tuple: A tuple containing:
                - k (numpy.ndarray): A 3D unit vector representing the axis of rotation. If no rotation is present (theta == 0), k is [0, 0, 0].
                - theta (float): The rotation angle in radians, representing the amount of rotation around the axis k.
        """
        # 计算旋转角度 theta
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta == 0: # 如果 theta 为零，意味着没有旋转，此时旋转轴向量 k 无意义
            k = np.array([0, 0, 0])
        else: # 如果 theta 不为零，使用以下公式计算旋转轴 k
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta

    def estimate_accuracy(self, R_sw_gt):
        """
        Evaluate the accuracy of the estimated rotation matrix by comparing it to the ground truth.

        This function computes the axis-angle difference between the estimated rotation matrix and 
        the ground truth rotation matrix. It returns the axis of rotation and the angular deviation (the closer to 0, the better).

        Args:
            R_sw_gt (numpy.ndarray): The ground truth rotation matrix, which is a 3x3 numpy array 
                                    representing the true rotation.

        Returns:
            tuple: A tuple containing:
                - axis (numpy.ndarray): A 3D unit vector representing the axis of rotation difference.
                - theta (float): The angular deviation between the estimated and ground truth rotation 
                                matrices, in radians.
        """
        axis, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return axis, theta
    

class NONAPPAlgorithm:
    
    def __init__(self):
        # t_s, R_sw
        self.R_sw = None
        self.t_s = None
        # 启动 MATLAB 引擎
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(script_dir)
        # self.eng.addpath('/home/clp/catkin_ws/src/lias_anp/scripts/anp/')
        print("Start matlab engine, ANP module successfully initialized!")
        
    def compute_t_R(self, P_SI, P_W):
        # 将 numpy 数组转换为 matlab.double 类型
        P_SI_matlab = matlab.double(P_SI.tolist())
        P_W_matlab = matlab.double(P_W.tolist())

        # 调用 MATLAB 中的 compute_t_R 函数
        R_sw, t_s = self.eng.Wang_nonapp_algorithm(P_W_matlab, P_SI_matlab, nargout=2)

        # 将结果保存到类的属性中
        self.R_sw = np.array(R_sw).T
        self.t_s = np.array(t_s)

        return self.t_s, self.R_sw

    @staticmethod
    def rot2aa(R):
        """
        Converts a 3x3 rotation matrix to axis-angle representation.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing a rotation in 3D space.

        Returns:
            tuple: A tuple containing:
                - k (numpy.ndarray): A 3D unit vector representing the axis of rotation. If no rotation is present (theta == 0), k is [0, 0, 0].
                - theta (float): The rotation angle in radians, representing the amount of rotation around the axis k.
        """
        # 计算旋转角度 theta
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta == 0: # 如果 theta 为零，意味着没有旋转，此时旋转轴向量 k 无意义
            k = np.array([0, 0, 0])
        else: # 如果 theta 不为零，使用以下公式计算旋转轴 k
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta

    def estimate_accuracy(self, R_sw_gt):
        """
        Evaluate the accuracy of the estimated rotation matrix by comparing it to the ground truth.

        This function computes the axis-angle difference between the estimated rotation matrix and 
        the ground truth rotation matrix. It returns the axis of rotation and the angular deviation (the closer to 0, the better).

        Args:
            R_sw_gt (numpy.ndarray): The ground truth rotation matrix, which is a 3x3 numpy array 
                                    representing the true rotation.

        Returns:
            tuple: A tuple containing:
                - axis (numpy.ndarray): A 3D unit vector representing the axis of rotation difference.
                - theta (float): The angular deviation between the estimated and ground truth rotation 
                                matrices, in radians.
        """
        axis, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return axis, theta

class APPAlgorithm:
    
    def __init__(self):
        # t_s, R_sw
        self.R_sw = None
        self.t_s = None
        # 启动 MATLAB 引擎
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath(script_dir)
        # self.eng.addpath('/home/clp/catkin_ws/src/lias_anp/scripts/anp/')
        print("Start matlab engine, ANP module successfully initialized!")
        
    def compute_t_R(self, P_SI, P_W, t_true, Var_noise=0):
        t_true = t_true.reshape(-1,1)
        # 将 numpy 数组转换为 matlab.double 类型
        P_W_matlab = matlab.double(P_W.tolist())
        P_SI_matlab = matlab.double(P_SI.tolist())
        t_true_matlab = matlab.double(t_true.tolist())

        # 调用 MATLAB 中的 compute_t_R 函数
        R_sw, t_s = self.eng.Wang_app_algorithm(P_W_matlab, P_SI_matlab, t_true_matlab, Var_noise, nargout=2)

        # 将结果保存到类的属性中
        self.R_sw = np.array(R_sw).T
        self.t_s = np.array(t_s)

        return self.t_s, self.R_sw

    @staticmethod
    def rot2aa(R):
        """
        Converts a 3x3 rotation matrix to axis-angle representation.

        Args:
            R (numpy.ndarray): A 3x3 rotation matrix representing a rotation in 3D space.

        Returns:
            tuple: A tuple containing:
                - k (numpy.ndarray): A 3D unit vector representing the axis of rotation. If no rotation is present (theta == 0), k is [0, 0, 0].
                - theta (float): The rotation angle in radians, representing the amount of rotation around the axis k.
        """
        # 计算旋转角度 theta
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta == 0: # 如果 theta 为零，意味着没有旋转，此时旋转轴向量 k 无意义
            k = np.array([0, 0, 0])
        else: # 如果 theta 不为零，使用以下公式计算旋转轴 k
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta

    def estimate_accuracy(self, R_sw_gt):
        """
        Evaluate the accuracy of the estimated rotation matrix by comparing it to the ground truth.

        This function computes the axis-angle difference between the estimated rotation matrix and 
        the ground truth rotation matrix. It returns the axis of rotation and the angular deviation (the closer to 0, the better).

        Args:
            R_sw_gt (numpy.ndarray): The ground truth rotation matrix, which is a 3x3 numpy array 
                                    representing the true rotation.

        Returns:
            tuple: A tuple containing:
                - axis (numpy.ndarray): A 3D unit vector representing the axis of rotation difference.
                - theta (float): The angular deviation between the estimated and ground truth rotation 
                                matrices, in radians.
        """
        axis, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return axis, theta