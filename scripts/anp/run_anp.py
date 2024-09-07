import numpy as np
import matlab.engine
from anp_alg_matlab import SonarDataGenerator, AnPAlgorithm
# from anp_alg import SonarDataGenerator, AnPAlgorithm
from pathlib import Path

# 获取脚本的路径
script_path = Path(__file__).resolve()

# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"


if __name__ == "__main__":
    # # 初始化参数
    # P_W = np.array([[30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15],
    #                 [44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42],
    #                 [35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47]])
    # R_SW = np.array([[-0.5798, 0.4836, -0.6557],
    #                 [-0.8135, -0.3883, 0.4329],
    #                 [-0.0453, 0.7844, 0.6186]])
    # t_S = np.array([6, 4, 8])
    
    # # 模拟生成 P_SI 和 P_SI_Noise 数据
    # data_generator = SonarDataGenerator(P_W, R_SW, t_S, Var_Noise=0.1)
    # P_S, P_SI, P_SI_Noise = data_generator.generate_data()

    # Defining the arrays as numpy arrays
    P_SI = np.array([
        [-34.28415699, -23.4147263, 30.58719872, -71.36205414, -62.98350553, -83.66340331, -54.11706308, -27.53627976, -35.66165012, -33.8476791, -67.36441633],
        [-58.46019646, -46.09452045, -67.72413967, 15.58309519, 5.06912889, -47.81114677, -18.30154963, -67.59046453, -65.16200224, -73.65466778, -21.42185885]
    ])

    P_W = np.array([
        [30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15],
        [44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42],
        [35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47]
    ])
    
    # 实例化数据生成器

    # 生成数据
    
    # 实例化算法
    anp_algorithm = AnPAlgorithm()

    # 计算 t_s 和 R_SW_Noise_my

    t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(P_SI, P_W)

    print("t_s_cal: \n", t_s_cal)
    print("R_sw_cal: \n", R_sw_cal)

    # 估计精度
    k, theta = anp_algorithm.estimate_accuracy(R_sw_cal)

    print("估计的精度 theta:", theta)

