import numpy as np
import matlab.engine
# from anp_alg_matlab import AnPAlgorithmMatlab
from anp_alg import AnPAlgorithmPython, AnPAlgorithmMatlab, NONAPPAlgorithm, APPAlgorithm
from sonar_data_generator import SonarDataGenerator
from pathlib import Path


# 获取脚本的路径
script_path = Path(__file__).resolve()

# 获取脚本所在的目录
script_dir = script_path.parent
script_dir = str(script_dir) + "/"


if __name__ == "__main__":   

    # 初始化参数
    
    P_W = np.array([[-3.85933012e-01,  4.25515244e+00,  4.24881689e-02,
         3.49546550e+00,  2.77211928e+00,  1.81099010e+00,
         2.44828144e+00,  3.52073216e+00,  2.29037279e+00,
         2.84096242e+00,  3.24242182e+00,  1.34012540e+00,
         2.61560941e+00,  9.75360572e-01,  2.16470295e+00,
         2.71682882e+00,  4.14370632e+00,  3.96442311e+00,
         4.78511423e-01,  3.47112790e+00,  5.60949445e-01,
         9.79993284e-01,  3.07926529e+00,  7.03669380e-01,
         3.75099707e+00,  3.62829018e+00,  3.79376698e+00,
         3.32001343e+00,  1.61765524e+00,  3.05458355e+00,
         3.14745641e+00,  2.89095121e+00,  2.58892640e+00,
         4.06969976e+00,  1.64160299e+00,  4.03649235e+00,
         2.13997769e+00,  2.69664502e+00,  3.73117360e+00,
         2.25690651e+00,  2.83105421e+00,  1.27759933e+00],
       [ 3.03557158e-01,  1.31931943e-02,  1.77477568e-01,
         1.40880830e+00,  2.23518276e+00, -6.11541629e-01,
         1.58268169e+00, -2.75001121e+00,  1.06248250e+00,
        -9.57536144e-01, -1.96029683e+00, -9.95549356e-01,
        -6.35721028e-01,  8.80005896e-01,  6.64956541e-02,
         1.93223803e+00,  1.45108283e+00,  9.52350281e-02,
         6.89194679e-01, -3.82512618e-01,  3.46914530e-01,
         1.24226642e+00, -9.29639083e-01, -9.51357364e-01,
         1.38880348e+00,  8.74414682e-01, -4.58376133e-01,
        -1.66372266e+00,  1.21157371e+00, -2.05404186e+00,
         2.34593058e+00, -8.08872379e-01, -2.36897507e-01,
         1.12665963e+00,  1.71046722e+00,  2.16458106e+00,
         1.94252282e-01, -1.98036933e+00, -1.23056902e+00,
         1.66774082e+00,  1.11162007e+00, -1.03249419e+00],
       [-1.99721567e-02,  5.89867813e-01,  3.02828789e-01,
         9.44239620e-01, -4.10095928e-03,  4.79895361e-02,
         5.87579031e-01,  2.42498055e-01,  8.37406081e-01,
         3.79846785e-01,  3.04300565e-01,  6.03391869e-01,
         8.53728130e-02,  7.71727934e-02,  8.79489220e-01,
         6.12135797e-01, -8.94514471e-03,  9.10840680e-01,
         3.72751117e-01,  3.19032060e-01, -7.82561526e-02,
        -9.07592922e-02,  8.29617211e-01,  2.94403815e-01,
         1.50186449e-01,  2.16394197e-02,  8.96943825e-01,
         9.28712964e-01,  4.17513879e-01,  1.53098613e-01,
         9.67515707e-01,  6.78574121e-01,  8.61552533e-01,
         1.79433048e-01,  3.14782888e-01,  8.74459296e-02,
         7.41949230e-02,  4.72494274e-01,  1.44109500e-01,
         4.19344574e-01,  2.16644451e-01,  9.52370405e-01]])

    R_SW = np.array([[ 0.98011915,  0.12109255, -0.15717206],
       [-0.10256481,  0.98732824,  0.12109255],
       [ 0.16984382, -0.10256481,  0.98011915]])
    
    t_S = np.array([-1.52132879, 0.444879,-0.2606644 ])
    
    print(-R_SW@t_S)
    
    # T = np.array([[0.98007119, 0.19820539, 0.0132322, -1.99112736], 
    #                 [-0.19686029, 0.96018782, 0.19820539, 0.13173626], 
    #                 [0.02657998, -0.19686029, 0.98007119,-0.49556368],
    #                 [0, 0, 0, 1]])
    # R = T[0:3,0:3]
    # t = T[0:3,3]
    # print(-R@t)
    
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    data_generator = SonarDataGenerator(P_W, R_SW, t_S)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    

    
    anp_algorithm_python = AnPAlgorithmPython()
    anp_algorithm_matlab = AnPAlgorithmMatlab()
    nonapp_algorithm = NONAPPAlgorithm()
    app_algorithm = APPAlgorithm()
    
    print("When there is no noise\n")
    t_s_cal0, R_sw_cal0 = anp_algorithm_python.compute_t_R(P_SI, P_W)
    t_s_cal1, R_sw_cal1 = anp_algorithm_matlab.compute_t_R(P_SI, P_W)
    t_s_cal2, R_sw_cal2 = nonapp_algorithm.compute_t_R(P_SI, P_W)
    t_s_cal3, R_sw_cal3 = app_algorithm.compute_t_R(P_SI, P_W, -R_SW@t_S, 10e-2)
    
    print("ANP old R error: ", anp_algorithm_python.estimate_accuracy(R_SW))
    print("ANP new R error: ", anp_algorithm_matlab.estimate_accuracy(R_SW))
    print("NONAPP R error: ", nonapp_algorithm.estimate_accuracy(R_SW))
    print("APP R error: ", app_algorithm.estimate_accuracy(R_SW))

    print("====================================")
    
    print("ANP old t error: ", np.linalg.norm(t_s_cal0.T-t_S))
    print("ANP new t error: ", np.linalg.norm(t_s_cal1.T-t_S))
    print("NONAPP t error: ", np.linalg.norm(t_s_cal2.T-t_S))
    print("APP t error: ", np.linalg.norm(t_s_cal3.T-t_S))




    print("\nWhen data has noise\n")
    anp_algorithm = AnPAlgorithmPython()
    t_s_cal0, R_sw_cal0 = anp_algorithm_python.compute_t_R(P_SI_Noise, P_W)
    t_s_cal1, R_sw_cal1 = anp_algorithm_matlab.compute_t_R(P_SI_Noise, P_W)
    t_s_cal2, R_sw_cal2 = nonapp_algorithm.compute_t_R(P_SI_Noise, P_W)
    t_s_cal3, R_sw_cal3 = app_algorithm.compute_t_R(P_SI_Noise, P_W, -R_SW*t_S, 10e-2)
    
    print("ANP old R error: ", anp_algorithm_python.estimate_accuracy(R_SW))
    print("ANP new R error: ", anp_algorithm_matlab.estimate_accuracy(R_SW))
    print("NONAPP R error: ", nonapp_algorithm.estimate_accuracy(R_SW))
    print("APP R error: ", app_algorithm.estimate_accuracy(R_SW))

    print("====================================")

    print("ANP old t error: ", np.linalg.norm(t_s_cal0.T-t_S))
    print("ANP new t error: ", np.linalg.norm(t_s_cal1.T-t_S))
    print("NONAPP t error: ", np.linalg.norm(t_s_cal2.T-t_S))
    print("APP t error: ", np.linalg.norm(t_s_cal3.T-t_S))
   

