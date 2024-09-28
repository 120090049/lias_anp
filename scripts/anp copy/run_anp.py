import numpy as np
import matlab.engine
from anp_alg_matlab import AnPAlgorithmMatlab
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
    P_W = np.array([[ 1.06468565,  2.31759309,  0.35106129,  3.633026  , -0.30355716,
        -1.99571699, -0.17747571,  1.88155627, -1.5290993 , -1.72765435,
         3.16261278,  4.63730969,  2.74471633,  3.97951076, -0.14802812,
         4.43218987,  4.04127959, -0.26189836,  0.912736  ,  2.6847464 ,
         0.3559706 , -0.81076358, -0.25064624, -1.85310636,  0.4857652 ,
         2.9175314 ,  2.15640638, -0.71552746,  3.78229859, -0.26303752,
         2.40594241,  2.68300674, -2.03231184,  0.5881672 ,  0.17541774],
       [ 2.15335171,  2.22427009,  0.13722424,  1.62571457,  2.30703358,
         1.62333665,  2.5212474 ,  2.93972395,  0.29379445,  0.85940493,
         0.78559129,  0.75710126,  2.36389638,  1.58152865,  1.96476345,
         0.70284667,  0.80582551,  2.11508347,  0.17749674,  0.90879937,
         2.60368177,  2.55750701,  2.09008228,  0.09527522,  0.07861022,
         2.42931478,  0.36894649,  0.96179572,  0.60865037,  1.34471242,
         2.44394798,  1.48191296,  2.30470538,  0.72382554,  2.55085797],
       [ 0.4460043 , -0.92815562, -0.36306902,  0.02044748, -0.01997227,
        -0.66784918,  0.30280853, -0.58435965, -0.4538    , -0.83419368,
         0.38298784, -0.31159393,  0.32168943, -0.10436606, -0.71233136,
        -0.93288816,  0.27015596, -0.99035812, -0.77765151, -0.565299  ,
        -0.38056115, -0.89752559, -0.637542  , -0.7976791 , -0.17864977,
         0.17748862, -0.25419538, -0.21643761, -0.83896555, -0.04488033,
        -0.14467027,  0.34145294, -0.1916855 , -0.94122864, -0.19017877]])
    
    R_SW = np.array([[ 0.99015653, -0.09001756,  0.10717691],
                    [ 0.10717691,  0.98012199, -0.16695508],
                    [-0.09001756,  0.17679855,  0.98012199]])
    
    t_S = np.array([-0.562109, 1.41947194, 0.58107842])
    
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    data_generator = SonarDataGenerator(P_W, R_SW, t_S)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    
    ##################################
    ## PYTHON ##
    ##################################
    print(R_SW)
    
    print("When there is no noise\n")
    print("PYTHON compare baseline")
    anp_algorithm = AnPAlgorithm()
    t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(P_SI, P_W)
    # print(t_s_cal)
    print("PYTHON:", R_sw_cal)
    print("R error: \n", anp_algorithm.estimate_accuracy(R_SW))
    print("t error: \n", np.linalg.norm(t_s_cal.T-t_S))

    
    ##################################
    ## MATLAB ##
    ##################################
    print("MATLAB for our new algorithm")
    anp_algorithm_matlab = AnPAlgorithmMatlab()
    t_s_cal, R_sw_cal = anp_algorithm_matlab.compute_t_R(P_SI, P_W)
    print("MATLAB:", R_sw_cal)
    print("R error: \n", anp_algorithm_matlab.estimate_accuracy(R_SW))
    print("t error: \n", np.linalg.norm(t_s_cal.T-t_S))


#######################################################################
#######################################################################
#######################################################################

    print("When data has noise\n")
    print("PYTHON compare baseline")
    anp_algorithm = AnPAlgorithm()
    t_s_cal, R_sw_cal = anp_algorithm.compute_t_R(P_SI_Noise, P_W)
    print("R error: \n", anp_algorithm.estimate_accuracy(R_SW))
    print("t error: \n", np.linalg.norm(t_s_cal.T-t_S))

    
    ##################################
    ## MATLAB ##
    ##################################
    print("MATLAB for our new algorithm")
    anp_algorithm_matlab = AnPAlgorithmMatlab()
    t_s_cal, R_sw_cal = anp_algorithm_matlab.compute_t_R(P_SI_Noise, P_W)
    print("R error: \n", anp_algorithm_matlab.estimate_accuracy(R_SW))
    print("t error: \n", np.linalg.norm(t_s_cal.T-t_S))

    # 估计精度
    # k, theta = anp_algorithm_matlab.estimate_accuracy(R_SW)
    # print("估计的精度 theta:", theta)

