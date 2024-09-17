import numpy as np

class SonarDataGenerator:
    def __init__(self, P_W, R_SW, t_S, theta_noise=0.0005, d_noise=0.0002):
        self.P_W = P_W
        self.R_SW = R_SW
        self.t_S = t_S
        self.theta_noise = theta_noise
        self.d_noise = d_noise
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
        P_SI_Noise = np.zeros((2, self.n))

        for i in range(self.n):
            # P_S[:, i] = self.R_SW @ self.P_W[:, i] + self.t_S
            P_S[:, i] = self.R_SW @ (self.P_W[:, i] - self.t_S)
            d = np.linalg.norm(P_S[:, i])
            tan_theta = P_S[1, i] / P_S[0, i]
            theta = np.arctan(tan_theta)
            P_SI[0, i] = d * np.cos(theta)
            P_SI[1, i] = d * np.sin(theta)
            
            tan_theta_noise = tan_theta + np.random.normal(0, self.theta_noise)
            theta_noise = np.arctan(tan_theta_noise)
            d_noise = d + np.random.normal(0, self.d_noise)
            P_SI_Noise[0, i] = d_noise * np.sin(theta_noise)
            P_SI_Noise[1, i] = d_noise * np.cos(theta_noise)
            #############################
            #############################
            
            # cos_theta[i] = P_S[0, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
            # sin_theta[i] = P_S[1, i] / np.sqrt(P_S[0, i]**2 + P_S[1, i]**2)
            # tan_theta[i] = sin_theta[i] / cos_theta[i]
            # theta[i] = np.arctan(tan_theta[i])
            # cos_phi[i] = np.sqrt(P_S[0, i]**2 + P_S[1, i]**2) / d[i]
            # P_SI[0, i] = d[i] * cos_theta[i]
            # P_SI[1, i] = d[i] * sin_theta[i]
        
        # theta_noise = np.arctan(np.tan(theta) + np.random.normal(0, self.theta_noise, size=Y.shape))
        # Rho_noise = np.sqrt(X**2 + Y**2 + Z**2) + np.random.normal(0, self.Rho_noise, size=Y.shape)
        # ps_x_noise = Rho_noise * np.sin(theta_noise)
        # ps_y_noise = Rho_noise * np.cos(theta_noise)
        # P_SI_Noise = P_SI + self.Var_Noise * np.random.randn(2, self.n)
        
        return P_S, P_SI, P_SI_Noise


if __name__ == "__main__":
    # 初始化参数
    # P_W = np.array([[30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15],
    #                 [44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42],
    #                 [35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47]])
    
    # R_SW = np.array( [[0.689673028293550, 0.423447870703166, 0.587403621746887],
    #                   [0.176155229057263, 0.688715772969376, -0.703306419236294],
    #                   [-0.702367745073895, 0.588525687510876, 0.400396136119796]])
    
    # t_S = np.array([6, 4, 8])
    
    # # 模拟生成 P_SI 和 P_SI_Noise 数据
    # data_generator = SonarDataGenerator(P_W, R_SW, t_S, Var_Noise=0.1)
    # P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    
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
    data_generator = SonarDataGenerator(P_W, R_SW, t_S, Var_Noise=0.1)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    print(P_SI)