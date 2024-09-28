import numpy as np
import matlab.engine

from pathlib import Path

# 获取脚本的路径
script_path = Path(__file__).resolve()

# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"



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


