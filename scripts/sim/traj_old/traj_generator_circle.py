#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class TrajectoryGenerator:
    def __init__(self, rate=1.0, delta_t=0.01):
        """
        初始化轨迹生成器参数
        :param a: 水平方向振幅
        :param b: 垂直方向振幅
        :param c: Z轴振幅
        :param rate: 轨迹速度
        :param delta_t: 时间步长
        """
        self.a = -1
        self.b = 1
        self.z_amplitude = 0.3
        self.rate = rate
        self.delta_t = delta_t
        self.t = 0.0

        self.r = 3
        self.omega = rate
        self.omega_z = 3*rate
        # 初始化ROS节点和发布器
        rospy.init_node('sonar_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/set_sonar_pose', PoseStamped, queue_size=10)
        self.rate_hz = rospy.Rate(10)  # 10 Hz

    def generate_trajectory(self, t):
        """
        生成8字形轨迹, 并计算 roll, pitch, yaw
        :param t: 时间
        :return: (x, y, z, roll, pitch, yaw)
        """
        # # 当前时刻的位置
        # x = self.a * np.sin(self.rate * t) - 1
        # y = self.b * np.sin(self.rate * t) * np.cos(self.rate * t)
        # z = self.c * np.sin(self.rate * t)

        # # 使用 delta_t 近似计算轨迹的导数（切线向量）
        # x_next = self.a * np.sin(self.rate * (t + self.delta_t)) - 1
        # y_next = self.b * np.sin(self.rate * (t + self.delta_t)) * np.cos(self.rate * (t + self.delta_t))
        # z_next = self.c * np.sin(self.rate * (t + self.delta_t))

        # 当前时刻的位置（螺旋轨迹）
        x = self.r * np.cos(self.omega * t)
        y = self.r * np.sin(self.omega * t)
        z = self.z_amplitude * np.sin(self.omega_z * t)

        # 使用 delta_t 近似计算轨迹的导数（切线向量）
        x_next = self.r * np.cos(self.omega * (t + self.delta_t))
        y_next = self.r * np.sin(self.omega * (t + self.delta_t))
        z_next = self.z_amplitude * np.sin(self.omega_z * (t + self.delta_t))
        
        # 切线向量的分量
        dx_dt = (x_next - x) / self.delta_t
        dy_dt = (y_next - y) / self.delta_t
        dz_dt = (z_next - z) / self.delta_t

        # 归一化切线向量
        magnitude = np.sqrt(dx_dt**2 + dy_dt**2 + dz_dt**2)
        tangent_x = dx_dt / magnitude
        tangent_y = dy_dt / magnitude
        tangent_z = dz_dt / magnitude

        # 计算 roll, pitch, yaw
        roll = 0  # 假设 roll 为 0
        pitch = -np.arctan2(tangent_z, np.sqrt(tangent_x**2 + tangent_y**2))  # 计算俯仰角 pitch
        yaw = np.arctan2(tangent_y, tangent_x)  # 计算航向角 yaw

        return x, y, z, roll, pitch, yaw

    def publish_pose(self):
        while not rospy.is_shutdown():
            # 生成当前位置和姿态
            x, y, z, roll, pitch, yaw = self.generate_trajectory(self.t)
            self.t += self.delta_t  # 时间递增

            # 创建并发布姿态消息
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.pub.publish(pose)
            self.rate_hz.sleep()

if __name__ == '__main__':
    try:
        # 初始化轨迹生成器，参数可以在这里修改
        traj_gen = TrajectoryGenerator(rate=1.0, delta_t=0.01)
        traj_gen.publish_pose()
    except rospy.ROSInterruptException:
        pass
