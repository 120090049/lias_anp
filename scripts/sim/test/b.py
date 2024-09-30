#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class CarTrajectory:
    def __init__(self, speed=1, trajectory_type='circle'):
        self.speed = speed
        self.time = 0
        self.trajectory_type = trajectory_type

    def update(self, delta_time):
        self.time += delta_time

    def get_pose(self):
        if self.trajectory_type == 'circle':
            return self.circle_trajectory()
        elif self.trajectory_type == 'square':
            return self.square_trajectory()
        elif self.trajectory_type == 'figure_8':
            return self.figure_8_trajectory()
        else:
            raise ValueError("Unsupported trajectory type")

    def circle_trajectory(self, radius=10):
        angle = self.speed * self.time / radius
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = 0
        return x, y, z

    def square_trajectory(self, side_length=10):
        period = 4 * side_length / self.speed
        t = self.time % period

        if t < side_length / self.speed:
            x = self.speed * t
            y = 0
        elif t < 2 * side_length / self.speed:
            x = side_length
            y = self.speed * (t - side_length / self.speed)
        elif t < 3 * side_length / self.speed:
            x = side_length - self.speed * (t - 2 * side_length / self.speed)
            y = side_length
        else:
            x = 0
            y = side_length - self.speed * (t - 3 * side_length / self.speed)

        z = 0
        return x, y, z

    def figure_8_trajectory(self, radius=5):
        angle = self.speed * self.time / radius
        x = radius * np.sin(angle)
        y = radius * np.sin(angle) * np.cos(angle)
        z = 0
        return x, y, z

def publish_trajectory():
    rospy.init_node('car_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/car_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    car = CarTrajectory(speed=1, trajectory_type='figure_8')

    while not rospy.is_shutdown():
        x, y, z = car.get_pose()
        car.update(0.1)

        # 创建并发布PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0  # 简单起见，只设置了一个单位四元数

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
