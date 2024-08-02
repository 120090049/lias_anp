#!/usr/bin/python3
import rospy
import numpy as np
from lias_anp.msg import SonarData  # 确保替换为你的包名
import csv

# This is the function implement in the package
from determinant import compute_D
from ..anp import AnPAlgorithm

class Determinant_Analysis:
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
        if len(data.indices) > 0:
            self.pose = data.pose
            self.w_p = np.array(data.w_p).reshape(-1, 3)
            self.s_p = np.array(data.s_p).reshape(-1, 3)
            self.si_q = np.array(data.si_q).reshape(-1, 2)
            self.si_q_theta_Rho = np.array(data.si_q_theta_Rho).reshape(-1, 2)
            self.timestep = data.timestep
            self.pts_indice = np.array(data.indices)

            # 写入文件
            # 将数据写入CSV文件
            with open("sonar_data.csv", 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.pose.position.x, self.pose.position.y, self.pose.position.z,
                                    self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w,
                                    self.w_p.tolist(), self.s_p.tolist(), self.si_q.tolist(), self.si_q_theta_Rho.tolist(),
                                    self.timestep, self.pts_indice.tolist()])
   
    def main_process(self):        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    tri_sim = Determinant_Analysis()
    tri_sim.main_process()

  