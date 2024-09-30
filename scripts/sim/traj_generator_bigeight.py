#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class TrajectoryGenerator:
    def __init__(self, rate=0.1, delta_t=0.01):
        """
        初始化轨迹生成器参数
        :param a: 水平方向振幅
        :param b: 垂直方向振幅
        :param c: Z轴振幅
        :param rate: 轨迹速度
        :param delta_t: 时间步长
        """
        self.rate = rate
        self.a = -3
        self.b = 3
        self.z_amplitude = 0.3
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
        x = self.a * np.sin(self.rate * t)-1
        y = self.b * np.sin(self.rate * t) * np.cos(self.rate * t)
        z = 0.5 * np.sin(self.rate * t)
        roll = 0.2 * np.sin(self.rate * t)
        pitch = 0.2 * np.cos(self.rate * t)
        yaw = 0.2 * np.sin(self.rate * t)
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
