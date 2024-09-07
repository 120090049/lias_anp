import numpy as np
import matlab.engine

from pathlib import Path

# 获取脚本的路径
script_path = Path(__file__).resolve()

# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"

class SonarDataGenerator:
    def __init__(self, P_W, R_SW, t_S, Var_Noise=1.0):
        self.P_W = P_W
        self.R_SW = R_SW
        self.t_S = t_S
        self.Var_Noise = Var_Noise
        self.n = P_W.shape[1]

    def generate_data(self):
        P_S = np.zeros((3, self.n))
        d = np.zeros(self.n)
        cos_theta = np.zeros(self.n)
        sin_theta = np.zeros(self.n)
        tan_theta = np.zeros(self.n)
        theta = np.zeros(self.n)
        cos_phi = np.zeros(self.n)
        P_SI = np.zeros((2, self.n))

        for i in range(self.n):
            P_S[:, i] = self.R_SW @ self.P_W[:, i] + self.t_S
            d[i] = np.linalg.norm(P_S[:, i])
            cos_theta[i] = P_S[0, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
            sin_theta[i] = P_S[1, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
            tan_theta[i] = sin_theta[i] / cos_theta[i]
            theta[i] = np.arctan(tan_theta[i])
            cos_phi[i] = np.sqrt(P_S[0, i]**2 + P_S[1, i]**2) / d[i]
            P_SI[0, i] = d[i] * cos_theta[i]
            P_SI[1, i] = d[i] * sin_theta[i]
        
        P_SI_Noise = P_SI + self.Var_Noise * np.random.randn(2, self.n)
        return P_S, P_SI, P_SI_Noise


class AnPAlgorithm:
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
        self.R_sw = np.array(R_sw)
        self.t_s = np.array(t_s)

        return self.t_s, self.R_sw

    @staticmethod
    def rot2aa(R):
        theta = np.arccos((np.trace(R) - 1) / 2)
        if theta == 0:
            k = np.array([0, 0, 0])
        else:
            k = np.array([(R[2, 1] - R[1, 2]),
                          (R[0, 2] - R[2, 0]),
                          (R[1, 0] - R[0, 1])]) / (2 * np.sin(theta))
        return k, theta

    def estimate_accuracy(self, R_sw_gt):
        k, theta = self.rot2aa(R_sw_gt.T @ self.R_sw)
        return k, theta

if __name__ == "__main__":
    # 初始化参数
    P_W = np.array([[30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15],
                    [44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42],
                    [35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47]])
    R_SW = np.array([[-0.5798, 0.4836, -0.6557],
                    [-0.8135, -0.3883, 0.4329],
                    [-0.0453, 0.7844, 0.6186]])
    t_S = np.array([6, 4, 8])
    
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    P_SI_Noise = np.array([[0.5, 0.2, 0.8],
                           [0.3, 0.7, 0.6],
                           [0.1, 0.4, 0.9]])

    
    # 实例化数据生成器
    data_generator = SonarDataGenerator(P_W, R_SW, t_S, Var_Noise=0.1)

    # 生成数据
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    
    # 实例化算法
    anp_algorithm = AnPAlgorithm()

    # 计算 t_s 和 R_SW_Noise_my

    t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(P_SI_Noise, P_W)

    print("t_s_cal: \n", t_s_cal)
    print("R_sw_cal: \n", R_sw_cal)

    # 估计精度
    k, theta = anp_algorithm.estimate_accuracy(R_SW)

    print("估计的精度 theta:", theta)

