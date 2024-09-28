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
    
    P_W = np.array([[4.01398693, 4.14420807, 4.32659229, 3.57898031, 4.62524004, 
                    2.53439912, 2.37290791, 2.92203137, 4.74740917, 4.4339805, 
                    4.65539456, 2.69388094, 5.05178386, 2.2541195, 2.8000855, 
                    4.553101, 4.0377846, 3.54886843, 3.15894055, 3.86783644, 
                    3.225728, 4.81897145, 2.97160881, 2.66377559, 4.14797142, 
                    5.04479198, 4.91589477, 3.80825367, 2.33517812],
                    [3.32507861, 2.98633423, 0.36589002, 1.76739533, 3.40097916, 
                    1.94232965, 1.42191891, -0.59761111, -0.97792378, 2.89282256, 
                    -1.10248539, -0.27824162, -0.71949274, 0.4283563, 2.29046146, 
                    1.71617653, 0.45081748, -0.02527929, -0.57252434, -0.10329507, 
                    2.70341004, -0.19826834, -0.45016959, 0.11943769, 1.48413903, 
                    -0.38693323, 0.03323147, -0.87652375, 2.02522025],
                    [0.00793917, 0.11863106, 0.20218476, 0.45847291, 0.4032435, 
                    0.06235353, 0.34978184, -0.11087785, -0.38486538, 0.49727416, 
                    -0.50759619, -0.37549141, -0.51327746, 0.37311051, 0.11160917, 
                    0.14894067, 0.4736446, -0.13407187, 0.36255587, 0.45950438, 
                    0.50665149, -0.10995821, 0.20389852, 0.42015753, -0.28143116, 
                    -0.17127721, -0.17202155, -0.28689548, -0.04151965]])

    R_SW = np.array([[0.98007119, 0.19820539, 0.0132322], 
                    [-0.19686029, 0.96018782, 0.19820539], 
                    [0.02657998, -0.19686029, 0.98007119]])

    t_S = np.array([-1.99112736, 0.13173626, -0.49556368])
    
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    data_generator = SonarDataGenerator(P_W, R_SW, t_S)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    

    print(R_SW)
    
    anp_algorithm_python = AnPAlgorithmPython()
    anp_algorithm_matlab = AnPAlgorithmMatlab()
    nonapp_algorithm = NONAPPAlgorithm()
    app_algorithm = APPAlgorithm()
    
    print("When there is no noise\n")
    t_s_cal0, R_sw_cal0 = anp_algorithm_python.compute_t_R(P_SI, P_W)
    t_s_cal1, R_sw_cal1 = anp_algorithm_matlab.compute_t_R(P_SI, P_W)
    t_s_cal2, R_sw_cal2 = nonapp_algorithm.compute_t_R(P_SI, P_W)
    t_s_cal3, R_sw_cal3 = app_algorithm.compute_t_R(P_SI, P_W, -R_SW@t_S, 10e-2)
    print(t_s_cal3, R_sw_cal3 )
    
    print("ANP old R error: ", anp_algorithm_python.estimate_accuracy(R_SW))
    print("ANP new R error: ", anp_algorithm_matlab.estimate_accuracy(R_SW))
    print("NONAPP R error: ", nonapp_algorithm.estimate_accuracy(R_SW))
    print("APP R error: ", app_algorithm.estimate_accuracy(R_SW))


  
    print("ANP old t error: ", np.linalg.norm(t_s_cal0.T-t_S))
    print("ANP new t error: ", np.linalg.norm(t_s_cal1.T-t_S))
    print("NONAPP t error: ", np.linalg.norm(t_s_cal2.T-t_S))
    print("ANP t error: ", np.linalg.norm(t_s_cal3.T-t_S))




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


  
    print("ANP old t error: ", np.linalg.norm(t_s_cal0.T-t_S))
    print("ANP new t error: ", np.linalg.norm(t_s_cal1.T-t_S))
    print("NONAPP t error: ", np.linalg.norm(t_s_cal2.T-t_S))
    print("ANP t error: ", np.linalg.norm(t_s_cal3.T-t_S))
   

