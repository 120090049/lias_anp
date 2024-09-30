#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter', anonymous=True)
        self.x_data = []
        self.y_data = []

        # 创建订阅器，订阅 /car_pose 话题
        self.subscriber = rospy.Subscriber('/car_pose', PoseStamped, self.pose_callback)

        # 初始化 matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.line, = self.ax.plot([], [], 'b-', lw=2)
        
        plt.ion()  # 开启交互模式
        plt.show()

    def pose_callback(self, data):
        # 获取汽车位置
        x = data.pose.position.x
        y = data.pose.position.y

        # 保存数据
        self.x_data.append(x)
        self.y_data.append(y)

        # 清除并重新绘制
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)

        self.ax.relim()  # 重新计算数据范围
        self.ax.autoscale_view()  # 自动缩放
        self.fig.canvas.draw()  # 触发重绘
        self.fig.canvas.flush_events()  # 刷新事件循环

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
