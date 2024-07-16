#!/usr/bin/python3
import rospy
import numpy as np
from lias_anp.msg import SonarData  # 确保替换为你的包名
from geometry_msgs.msg import PoseStamped
from utils import pose_to_transform_matrix

def get_match_pairs(si_q_theta_Rho, pts_indice, si_q_theta_Rho_prime, pts_indice_prime):
    
    # 找到共同的索引
    common_indices = np.intersect1d(pts_indice, pts_indice_prime)

    # 获取t1时刻的匹配坐标
    t1_indices = [np.where(pts_indice == idx)[0][0] for idx in common_indices]
    matched_t1 = si_q_theta_Rho[t1_indices]

    # 获取t0时刻的匹配坐标
    t0_indices = [np.where(pts_indice_prime == idx)[0][0] for idx in common_indices]
    matched_t0 = si_q_theta_Rho_prime[t0_indices]
    
    return matched_t1, matched_t0

def compute_D(R, t, theta, theta_prime):
    """
    Compute the determinant D(A0; R, t) for given parameters.
    """
    def skew_symmetric_matrix(t):
        """
        Create a skew-symmetric matrix for a vector t.
        """
        return np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
    ux = np.array([1, 0, 0])
    uy = np.array([0, 1, 0])
    
    r1 = R[0, :]
    r2 = R[1, :]
        
    t_cross = skew_symmetric_matrix(t)
    
    determinant = - (r1 - np.tan(theta) * r2).T @ t_cross @ (ux - np.tan(theta_prime) * uy)
    
    return determinant

def sonar_triangulation(T1, T2, d1, theta1, d2, theta2):
    """
    T1: The first sonar pose w.r.t. the world frame, represented as a 4x4 matrix [R1, t1; 0, 1].
    T2: The second sonar pose w.r.t. the world frame, represented as a 4x4 matrix [R2, t2; 0, 1].
    d1: The distance measurement of the first sonar in meters.
    theta1: The angle measurement of the first sonar in radians.
    d2: The distance measurement of the second sonar in meters.
    theta2: The angle measurement of the second sonar in radians.
    Returns:
    x: The estimated coordinates of the 3D point in the first sonar frame.
    """
    # Calculate the relative transformation
    T = np.linalg.inv(T2) @ T1
    R = T[:3, :3]
    t = T[:3, 3]

    r1 = R[0, :]
    r2 = R[1, :]

    # Solve the first set of linear equations
    A1 = np.vstack([
        [np.tan(theta1), -1, 0],
        [np.tan(theta2) * r1 - r2],
        [t @ R]
    ])
    b1 = np.array([0, t[1] - np.tan(theta2) * t[0], (d2**2 - d1**2 - np.linalg.norm(t)**2) / 2])

    x = np.linalg.inv(A1) @ b1

    # Solve the second set of linear equations
    A2 = np.vstack([A1, x])
    b2 = np.append(b1, d1**2)

    x = np.linalg.inv(A2.T @ A2) @ A2.T @ b2

    return x

class TriSim:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('tri_sim', anonymous=True)
        
        # 定义订阅者
        self.sonar_data_sub = rospy.Subscriber('/sim/sonar_data_with_pose', SonarData, self.sonar_callback)
        # self.pose_sub = rospy.Subscriber('/sim/sonar_data_with_pose', PoseStamped, self.pose_callback)

        # 定义数据存储
        self.w_p = None
        self.s_p = None
        self.si_q = None
        self.timestep = None
        
        self.pts_indice = None
        self.si_q_theta_Rho = None
        
        self.pts_indice_prime = None
        self.si_q_theta_Rho_prime = None
        
        self.pose = None
        self.pose_T = None
        self.R = None
        self.t = None
    

    def sonar_callback(self, data):
        # 将消息数据转换为numpy数组
        self.w_p = np.array(data.w_p).reshape(-1, 3)
        self.s_p = np.array(data.s_p).reshape(-1, 3)
        self.si_q = np.array(data.si_q).reshape(-1, 2)
        self.si_q_theta_Rho = np.array(data.si_q_theta_Rho).reshape(-1, 2)
        self.timestep = data.timestep
        self.pts_indice = np.array(data.indices)
        
        if len(self.pts_indice) > 0:
            # 打印接收到的PoseStamped消息
            
            self.pose = data.pose
            self.pose_T = pose_to_transform_matrix(self.pose)
    
            with open('record.txt', 'a') as file:
                data = f"Pose: {self.pose_T.tolist()}; theta_Rho: {self.si_q_theta_Rho.tolist()}; s_p: {self.s_p.tolist()}; w_p: {self.w_p.tolist()}\n"
                file.write(data)

        
    # def sonar_callback(self, data):
    #     # 将消息数据转换为numpy数组
    #     self.w_p = np.array(data.w_p).reshape(-1, 3)
    #     self.s_p = np.array(data.s_p).reshape(-1, 3)
    #     self.si_q = np.array(data.si_q).reshape(-1, 2)
    #     self.si_q_theta_Rho = np.array(data.si_q_theta_Rho).reshape(-1, 2)
    #     self.timestep = data.timestep
    #     self.pts_indice = np.array(data.indices)
        
    #     if len(self.pts_indice) > 0:
    #         # 打印接收到的PoseStamped消息
    #         if self.pose_T is not None:
    #             T0 = self.pose_T
    #             self.pose = data.pose
    #             T1 = pose_to_transform_matrix(self.pose)   

    #             # Calculate the transition matrix from t0 to t1 using left multiplication
    #             delt_T = T1 @ np.linalg.inv(T0)
                
    #             self.R = delt_T[:3, :3]
    #             self.t = delt_T[:3, 3]
    #             self.pose_T = T1
    #             theta = self.si_q_theta_Rho[0][0]
    #             if self.si_q_theta_Rho_prime is not None:
    #                 theta_Rho, theta_Rho_prime = get_match_pairs(self.si_q_theta_Rho, self.pts_indice, self.si_q_theta_Rho_prime, self.pts_indice_prime)
    #                 if len(theta_Rho): 
    #                     determinant = compute_D(self.R, self.t, theta_Rho[0][0], theta_Rho_prime[0][0])
    #                     data = f"D: {determinant}, R: {self.R.tolist()}, t: {self.t.tolist()}, theta_Rho: {theta_Rho.tolist()}, theta_Rho_prime: {theta_Rho_prime.tolist()}\n"
        
    #                     if determinant > 0.01:
    #                         with open('non_deg.txt', 'a') as file:
    #                             file.write(data)
    #                         rospy.loginfo("determinant:\n%s\n", determinant)
    #                     else:
    #                         if not np.allclose(self.R, np.eye(3) , atol=0.03): # rotation
    #                             with open('deg_rot.txt', 'a') as file:
    #                                 file.write(data)
    #                             rospy.loginfo("deg_rot-determinant:\n%s\n", determinant)
                            
    #                         elif not np.allclose(self.t, np.zeros((3,)), atol=0.2):
    #                             with open('deg_trans.txt', 'a') as file:
    #                                 file.write(data)
    #                             rospy.loginfo("deg_trans-determinant:\n%s\n", determinant)
                            
    #         else:
    #             self.pose = data.pose
    #             self.pose_T = pose_to_transform_matrix(self.pose)
        
    #     self.pts_indice_prime = self.pts_indice
    #     self.si_q_theta_Rho_prime = self.si_q_theta_Rho 
        
        # 处理数据或执行其他操作
        # print(f"Timestep: {self.timestep}")
        # print(f"w_p: {self.w_p}")
        # print(f"s_p: {self.s_p}")
        # print(f"si_q: {self.si_q}")
        # print(f"si_q_theta_Rho: {self.si_q_theta_Rho}")

    def main_process(self):        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    tri_sim = TriSim()
    tri_sim.main_process()
