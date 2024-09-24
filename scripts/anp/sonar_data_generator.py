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
            # P_S[:, i] = self.R_SW @ self.P_W[:, i] + self.t_S
            P_S[:, i] = np.linalg.inv(self.R_SW) @ (self.P_W[:, i] - self.t_S)
            d = np.linalg.norm(P_S[:, i])
            tan_theta = P_S[1, i] / P_S[0, i]
            theta = np.arctan(tan_theta)
            P_SI[0, i] = d * np.cos(theta)
            P_SI[1, i] = d * np.sin(theta)
            
            tan_theta_noise = tan_theta + np.random.normal(0, self.theta_noise)
            theta_noise = np.arctan(tan_theta_noise)
            d_noise = d + np.random.normal(0, self.d_noise)
            P_SI_Noise[0, i] = d_noise * np.cos(theta_noise)
            P_SI_Noise[1, i] = d_noise * np.sin(theta_noise)
        
        return P_S, P_SI, P_SI_Noise
#########################



if __name__ == "__main__":

  P_W = np.array([[ 1.69730401, -1.91863542,  0.21188833],
       [-0.38593302,  0.30355716, -0.01997216]]).T
    
  pose = np.array([[ 0.98012376, -0.18085117, -0.08154915, -0.17158962],
       [ 0.16389133,  0.96975898, -0.18085117, -0.46401057],
       [ 0.11179017,  0.16389133,  0.98012376,  0.41420519],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
  R_SW = pose[:3,:3]
  
  t_S = pose[:3, 3]
  
  # 模拟生成 P_SI 和 P_SI_Noise 数据
  data_generator = SonarDataGenerator(P_W, R_SW, t_S)
  P_S, P_SI, P_SI_Noise = data_generator.generate_data()
  print(P_SI)
  
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