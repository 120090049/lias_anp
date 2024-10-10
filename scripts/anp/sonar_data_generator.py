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
        P_SI = np.zeros((2, self.n))
        P_SI_Noise = np.zeros((2, self.n))

        for i in range(self.n):

            P_S[:, i] = np.linalg.inv(self.R_SW) @ (self.P_W[:, i] - self.t_S)            
            d = np.linalg.norm(P_S[:, i])
            
            cos_theta = P_S[0, i] / np.linalg.norm(P_S[0:2, i])
            sin_theta = P_S[1, i] / np.linalg.norm(P_S[0:2, i])
            P_SI[0, i] = d * cos_theta
            P_SI[1, i] = d * sin_theta
            
            
            ###############################################################
            d_noise = d + np.random.normal(0, self.d_noise)
            
            tan_theta = P_S[1, i] / P_S[0, i]
            tan_theta_noise = tan_theta + np.random.normal(0, self.theta_noise)
            theta_noise = np.arctan(tan_theta_noise)
            # 保持符号一致
            original_sign_x = np.sign(P_S[0, i])
            original_sign_y = np.sign(P_S[1, i])
            # 计算新的 cos_theta 和 sin_theta
            cos_theta = original_sign_x * np.abs(np.cos(theta_noise))
            sin_theta = original_sign_y * np.abs(np.sin(theta_noise))

            # 确保 cos_theta 和 sin_theta 的平方和为1
            norm = np.sqrt(cos_theta**2 + sin_theta**2)
            cos_theta /= norm
            sin_theta /= norm
            P_SI_Noise[0, i] = d_noise * np.cos(theta_noise)
            P_SI_Noise[1, i] = d_noise * np.sin(theta_noise)
        
        return P_S, P_SI, P_SI_Noise
#########################



if __name__ == "__main__":

    P_W = np.array([[ 2.13603473, -0.77186602,  1.95072114,  1.82645702,  0.80637312,
         0.34877479,  1.43712997,  1.12189889,  1.95998657,  0.54235911,
         1.3069241 , -0.50389582, -0.77042562,  0.56313092,  2.09759974,
         1.93175828,  0.97354203,  1.29132247,  2.18199611,  0.91832501,
         1.60884869, -0.6045416 ,  2.45790458, -0.15580471,  1.31191647,
        -0.9142952 ,  2.26811886,  2.21745372, -0.04960047,  1.02438521,
         1.28548098,  2.00271249,  0.90937901],
       [ 1.6240803 ,  0.60711432,  1.76001179,  4.04454041,  2.19236112,
         0.5548203 ,  0.32810998,  0.69382906,  2.48453283,  1.49465704,
         3.06561542,  3.15025473,  2.14143443,  2.99997115,  4.09572172,
         0.40185145,  3.47797823,  0.44462281,  0.18353406,  1.21580398,
         2.54795504,  1.46164775,  1.08357716,  2.23380876,  2.69423199,
         2.478338  ,  1.88500488,  1.7469312 ,  1.33389342,  4.6142211 ,
        -0.0169981 ,  0.15571359,  4.31394911],
       [-1.36646855, -0.03994431,  0.15434559,  0.09593417, -0.25782672,
        -0.38742959, -0.24006221, -0.15651231, -0.18151858, -0.71256626,
        -0.83193976,  0.43299642, -0.08076227,  0.12128959,  0.3754721 ,
        -0.86207271,  0.56393188, -0.48081446, -0.20747855, -0.77819484,
        -1.04896271, -0.43491939, -0.75536293,  0.19425119, -0.68284667,
         0.39668149,  0.18036377, -1.16874087, -0.45922306, -0.33221188,
        -1.37119341, -1.44178295,  0.51926494]])

    T = np.array([[ 0.98013292, -0.12185598,  0.15649465, -3.15206827],
       [ 0.14159372,  0.98239622, -0.12185598,  1.4993604 ],
       [-0.13889087,  0.14159372,  0.98013292,  0.35867805],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
    
    R_SW = T[0:3, 0:3]
    t_S = T[0:3, 3].reshape(-1)
  
    # 模拟生成 P_SI 和 P_SI_Noise 数据
    data_generator = SonarDataGenerator(P_W, R_SW, t_S)
    P_S, P_SI, P_SI_Noise = data_generator.generate_data()
    print(P_SI)
  