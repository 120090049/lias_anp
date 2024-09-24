import numpy as np
from anp_alg import AnPAlgorithm
from sonar_data_generator import SonarDataGenerator
from pathlib import Path


# 获取脚本的路径
script_path = Path(__file__).resolve()

# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"


if __name__ == "__main__":
  
    # 初始化参数
    P_W = np.array([[ 4.56034272,  4.47827487, -0.88689726],
       [-4.15128005,  3.35498878,  0.47193998],
       [ 1.69730401, -1.91863542,  0.21188833],
       [ 1.06801734,  0.81204017, -0.68323426],
       [-0.6933036 , -1.0646818 ,  0.44602416],
       [ 4.94819563,  4.49395473,  0.08835409],
       [-0.55145811, -2.31759258, -0.92815134],
       [-4.72555143, -0.35106138, -0.36306974],
       [-1.19985078,  3.91789458,  0.05150554],
       [ 0.60510361, -2.63876593, -0.95228384],
       [-1.74857071, -3.63302607,  0.02044769],
       [ 4.98683568,  1.74479697, -0.63631301],
       [ 3.93571537,  2.96759921,  0.46880338],
       [ 4.0659365 ,  2.62885484,  0.57949527],
       [-1.46213022,  4.80976573,  0.92380188],
       [-3.38815347,  2.54004072,  0.4303018 ],
       [-0.38593302,  0.30355716, -0.01997216],
       [ 4.24832072,  0.00841063,  0.66304898],
       [-1.46075795,  3.82850919,  0.79940118],
       [-0.38987835,  0.6770507 ,  0.84066088]]).T
    
    R_SW = np.array([[ 0.99015653, -0.09001756,  0.10717691],
                    [ 0.10717691,  0.98012199, -0.16695508],
                    [-0.09001756,  0.17679855,  0.98012199]])
    
    t_S = np.array([-0.562109, 1.41947194, 0.58107842])
    
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    data_generator = SonarDataGenerator(P_W, R_SW, t_S)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    
    
    # T2_gt = np.array([[ 0.99987252, -0.01010873,  0.01235945,  0.05150115],
    #                 [ 0.01235945,  0.98006728, -0.19828103, -0.94843023],
    #                 [-0.01010873,  0.19840851,  0.98006728,  0.02578488],
    #                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
    # R_SW = T2_gt[:3,:3]
    # t_S = T2_gt[:3,3]
    print("When there is no noise\n")
    print("PYTHON compare baseline")
    anp_algorithm = AnPAlgorithm()
    t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(P_SI, P_W)
    print(t_s_cal.T)
    print(t_S)
    print()
    print(R_sw_cal)
    print(R_SW)
    print("R error: \n", anp_algorithm.estimate_accuracy(R_SW))
    print("t error: \n", np.linalg.norm(t_s_cal.T-t_S))

    