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
    
    P_W = np.array([[ 6.3723361 ,  4.5240402 ,  6.76254577,  6.25698193,  7.79260445,
         6.72762702,  7.60267654,  7.08325777,  7.66213854,  7.23866499,
         3.50257951,  6.4859005 ,  7.71081349,  5.81442691,  3.34292972,
         5.00638702,  7.20522162,  4.29802294,  7.5583895 ,  6.42820651],
       [-2.41570381, -0.69241184,  0.03362135, -0.81518972, -0.01676097,
        -0.46288979, -1.31933768, -0.73648026, -0.35025381, -2.76620607,
        -0.11115857, -0.76333301,  0.69313454, -1.26736277, -0.87691552,
         0.12607952, -2.14465185,  0.02662272,  1.64634791,  1.31753699],
       [-0.5415008 , -0.88246241, -0.11896007, -0.28181641, -0.90820135,
        -1.2712701 , -0.28216007, -1.4478447 , -1.30231427,  0.30042688,
        -0.54460507, -0.43337675, -0.42835824, -0.10344146, -0.4873707 ,
        -1.09733627, -0.60270704, -0.47601693, -0.47785228, -0.45450962]])

    T = np.array([[ 0.98006778,  0.19166309,  0.05227235,  1.99314574],
       [-0.19820337,  0.96123914,  0.19166309, -0.20221549],
       [-0.01351149, -0.19820337,  0.98006778, -0.49885762],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    # P_W = np.array([[ 2.13603473, -0.77186602,  1.95072114,  1.82645702,  0.80637312,
    #      0.34877479,  1.43712997,  1.12189889,  1.95998657,  0.54235911,
    #      1.3069241 , -0.50389582, -0.77042562,  0.56313092,  2.09759974,
    #      1.93175828,  0.97354203,  1.29132247,  2.18199611,  0.91832501,
    #      1.60884869, -0.6045416 ,  2.45790458, -0.15580471,  1.31191647,
    #     -0.9142952 ,  2.26811886,  2.21745372, -0.04960047,  1.02438521,
    #      1.28548098,  2.00271249,  0.90937901],
    #    [ 1.6240803 ,  0.60711432,  1.76001179,  4.04454041,  2.19236112,
    #      0.5548203 ,  0.32810998,  0.69382906,  2.48453283,  1.49465704,
    #      3.06561542,  3.15025473,  2.14143443,  2.99997115,  4.09572172,
    #      0.40185145,  3.47797823,  0.44462281,  0.18353406,  1.21580398,
    #      2.54795504,  1.46164775,  1.08357716,  2.23380876,  2.69423199,
    #      2.478338  ,  1.88500488,  1.7469312 ,  1.33389342,  4.6142211 ,
    #     -0.0169981 ,  0.15571359,  4.31394911],
    #    [-1.36646855, -0.03994431,  0.15434559,  0.09593417, -0.25782672,
    #     -0.38742959, -0.24006221, -0.15651231, -0.18151858, -0.71256626,
    #     -0.83193976,  0.43299642, -0.08076227,  0.12128959,  0.3754721 ,
    #     -0.86207271,  0.56393188, -0.48081446, -0.20747855, -0.77819484,
    #     -1.04896271, -0.43491939, -0.75536293,  0.19425119, -0.68284667,
    #      0.39668149,  0.18036377, -1.16874087, -0.45922306, -0.33221188,
    #     -1.37119341, -1.44178295,  0.51926494]])

    # T = np.array([[ 0.98013292, -0.12185598,  0.15649465, -3.15206827],
    #    [ 0.14159372,  0.98239622, -0.12185598,  1.4993604 ],
    #    [-0.13889087,  0.14159372,  0.98013292,  0.35867805],
    #    [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    R_SW = T[0:3, 0:3]
    t_S = T[0:3, 3].reshape(-1)
    
    
    # t_S = np.array([-1.52132879, 0.444879,-0.2606644 ])
    
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
   

