#!/usr/bin/python3
import rospy
import numpy as np
from lias_anp.msg import SonarData  # 确保替换为你的包名
import csv

class SonarDataWriter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('sonar_data_write', anonymous=True)
        
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
        
        # We need to set the frequency
        frequency = 5/7
        self.last_callback_time = rospy.Time.now()
        self.callback_interval = rospy.Duration(1/frequency)  # Throttle to 5 Hz
    
    
    def sonar_callback(self, data):
        current_time = rospy.Time.now()
        if current_time - self.last_callback_time >= self.callback_interval:
            self.last_callback_time = current_time
        
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

class SonarDataReader:
    def __init__(self, filepath):
        self.filepath = filepath
        self.data = []

    def read_data(self):
        with open(self.filepath, 'r') as file:
            reader = csv.reader(file)
            # headers = next(reader)  # 跳过表头

            for row in reader:
                pose_x = float(row[0])
                pose_y = float(row[1])
                pose_z = float(row[2])
                pose_orient_x = float(row[3])
                pose_orient_y = float(row[4])
                pose_orient_z = float(row[5])
                pose_orient_w = float(row[6])

                w_p = np.array(eval(row[7]))
                s_p = np.array(eval(row[8]))
                si_q = np.array(eval(row[9]))
                si_q_theta_Rho = np.array(eval(row[10]))
                timestep = int(row[11])
                pts_indice = np.array(eval(row[12]))

                self.data.append({
                    'pose': {
                        'position': {'x': pose_x, 'y': pose_y, 'z': pose_z},
                        'orientation': {'x': pose_orient_x, 'y': pose_orient_y, 'z': pose_orient_z, 'w': pose_orient_w}
                    },
                    'w_p': w_p,
                    's_p': s_p,
                    'si_q': si_q,
                    'si_q_theta_Rho': si_q_theta_Rho,
                    'timestep': timestep,
                    'pts_indice': pts_indice
                })

    def get_data(self):
        return self.data


if __name__ == '__main__':
    # Read
    sonar_data_write = SonarDataWriter()
    sonar_data_write.main_process()
    
    # Write
    # filepath = "sonar_data.csv"
    # reader = SonarDataReader(filepath)
    # reader.read_data()
    # data = reader.get_data()

    # # 打印读取的数据
    # for entry in data:
    #     print("Pose Position: ", entry['pose']['position'])
    #     print("Pose Orientation: ", entry['pose']['orientation'])
    #     print("w_p: ", entry['w_p'])
    #     print("s_p: ", entry['s_p'])
    #     print("si_q: ", entry['si_q'])
    #     print("si_q_theta_Rho: ", entry['si_q_theta_Rho'])
    #     print("Timestep: ", entry['timestep'])
    #     print("Pts Indice: ", entry['pts_indice'])
    #     print("\n")
    #     break


  