#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def generate_trajectory(t, a=2.0, b=2.0, c=0.5, rate=1):
    """
    Generate 8-shaped trajectory
    :param t: time
    :param a: horizontal amplitude
    :param b: vertical amplitude
    :param rate: speed of the trajectory
    :return: (x, y, z, roll, pitch, yaw)
    """
    
    x = a * np.sin(rate * t)
    y = b * np.sin(rate * t) * np.cos(rate * t)
    z = c * np.sin(rate * t)
    x_tangent = a * rate * np.cos(rate * t)
    y_tangent = b * rate * np.cos(2 * rate * t)
    z_tangent = c * rate * np.cos(rate * t)
    
    tangent = np.array([x_tangent, y_tangent, z_tangent])

    # 计算单位切线向量
    T_x, T_y, T_z = tangent
    yaw = np.arctan2(T_y, T_x)
    pitch = -np.arctan2(T_z, np.sqrt(T_x**2 + T_y**2))
    roll = 0
    
    return x, y, z, roll, pitch, yaw

# def generate_figure8_trajectory(t, a=1.0, b=1.0, rate=1):
#     """
#     Generate 8-shaped trajectory
#     :param t: time
#     :param a: horizontal amplitude
#     :param b: vertical amplitude
#     :param rate: speed of the trajectory
#     :return: (x, y, z, roll, pitch, yaw)
#     """
#     x = a * np.sin(rate * t)-1
#     y = b * np.sin(rate * t) * np.cos(rate * t)
#     z = 0.5 * np.sin(rate * t)
#     roll = 0.2 * np.sin(rate * t)
#     pitch = 0.2 * np.cos(rate * t)
#     yaw = 0.2 * np.sin(rate * t)
#     return x, y, z, roll, pitch, yaw

def main():
    rospy.init_node('sonar_pose_publisher')
    pub = rospy.Publisher('/sonar_pose_publisher', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    t = 0.0
    while not rospy.is_shutdown():
        # x, y, z, roll, pitch, yaw = generate_figure8_trajectory(t)
        x, y, z, roll, pitch, yaw = generate_trajectory(t)
        t += 0.01

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

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
