#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class TrajectoryGenerator:
    def __init__(self, rate=0.1, delta_t=0.01):
        """
        初始化轨迹生成器参数
        :param rate: 轨迹速度
        :param delta_t: 时间步长
        """
        self.rate = rate
        self.delta_t = delta_t
        self.t = 0.0

        self.side_length = 4  # 正方形边长
        self.corner_radius = 1  # 角落圆弧半径
        
        self.straight_length = self.side_length - 2 * self.corner_radius
        self.quarter_circle = np.pi * self.corner_radius / 2
        self.total_length = 4 * (self.straight_length + self.quarter_circle)
        self.period = self.total_length / self.rate  # 完成一圈所需的时间
        
        # 初始化ROS节点和发布器
        rospy.init_node('sonar_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/set_sonar_pose', PoseStamped, queue_size=10)
        self.rate_hz = rospy.Rate(10)  # 10 Hz

    def generate_trajectory(self, t):
        """
        生成正方形轨迹，角落为1/4圆弧，并计算 roll, pitch, yaw
        :param t: 时间
        :return: (x, y, z, roll, pitch, yaw)
        """
        
        t = t % self.period  # 确保t在一个周期内
        distance = (t * self.rate) % self.total_length  # 当前走过的总距离
        # distance += 2 * self.straight_length + self.quarter_circle
        # 确定当前在哪个段
        if distance < self.straight_length: # 1
            x = distance + self.corner_radius
            y = self.corner_radius
            yaw = 0
        elif distance < self.straight_length + self.quarter_circle: # 2
            angle = (distance - self.straight_length) / self.corner_radius
            x = self.side_length - self.corner_radius + self.corner_radius * np.sin(angle)
            y =  self.corner_radius * np.cos(angle)
            yaw = -angle
        elif distance < 2 * self.straight_length + self.quarter_circle: # 3
            x = self.side_length
            y = -(distance - self.straight_length - self.quarter_circle)
            yaw = -np.pi/2
        elif distance < 2 * self.straight_length + 2 * self.quarter_circle: # 4
            angle = (distance - (2 * self.straight_length + self.quarter_circle)) / self.corner_radius
            x = self.side_length - self.corner_radius + self.corner_radius * np.cos(angle)
            y = - self.straight_length - self.corner_radius * np.sin(angle)
            yaw = -(np.pi/2 + angle)
        elif distance < 3 * self.straight_length + 2 * self.quarter_circle: # 5
            x = self.corner_radius - distance +3 * self.straight_length + 2 * self.quarter_circle
            y = - self.straight_length - self.corner_radius
            yaw = -np.pi
        elif distance < 3 * self.straight_length + 3 * self.quarter_circle: # 6
            angle = (distance - (3 * self.straight_length + 2 * self.quarter_circle)) / self.corner_radius
            x = self.corner_radius - self.corner_radius * np.sin(angle)
            y = -self.straight_length - self.corner_radius * np.cos(angle)
            yaw = -(np.pi + angle)
        elif distance < 4 * self.straight_length + 3 * self.quarter_circle: # 7
            x = 0
            y = -(4*self.straight_length - distance + 3*self.quarter_circle)
            yaw = -3*np.pi/2
        else:
            angle = (distance - (4 * self.straight_length + 3 * self.quarter_circle)) / self.corner_radius
            x = self.corner_radius * (1-np.cos(angle))
            y = self.corner_radius * np.sin(angle)
            yaw = -(1.5*np.pi + angle)

        z = 0.5 + 0.1 * np.sin(2 * np.pi * t / self.period)
        roll = 0.1 * np.sin(2 * np.pi * t / self.period)
        pitch = 0.1 * np.cos(2 * np.pi * t / self.period)

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
        traj_gen = TrajectoryGenerator(rate=0.5, delta_t=0.01)
        traj_gen.publish_pose()
    except rospy.ROSInterruptException:
        pass