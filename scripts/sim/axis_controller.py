#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys
import select
import tty
import termios

class AxisController:
    def __init__(self, frame_id="map"):
        rospy.init_node("axis_controller")
        self.server = InteractiveMarkerServer("interactive_marker_example")
        self.pub = rospy.Publisher("/set_sonar_pose", PoseStamped, queue_size=10)
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = frame_id
        self.int_marker.name = "my_marker"
        self.int_marker.description = "Simple 6-DOF Control"
        self.int_marker.pose.position = Point(0, 0, 0)
        self.create_controls()
        self.server.insert(self.int_marker, self.process_feedback)
        self.server.applyChanges()
        self.speed = 0.1
        self.rotation_speed = 0.1

    def create_controls(self):
        axes = [
            ((1, 0, 0), "move_x", InteractiveMarkerControl.MOVE_AXIS),
            ((0, 1, 0), "move_y", InteractiveMarkerControl.MOVE_AXIS),
            ((0, 0, 1), "move_z", InteractiveMarkerControl.MOVE_AXIS),
            ((1, 0, 0), "rotate_x", InteractiveMarkerControl.ROTATE_AXIS),
            ((0, 1, 0), "rotate_y", InteractiveMarkerControl.ROTATE_AXIS),
            ((0, 0, 1), "rotate_z", InteractiveMarkerControl.ROTATE_AXIS)
        ]
        for orientation, name, mode in axes:
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x, control.orientation.y, control.orientation.z = orientation
            control.name = name
            control.interaction_mode = mode
            self.int_marker.controls.append(control)

    def process_feedback(self, feedback):
        self.int_marker.pose = feedback.pose
        self.server.applyChanges()
        self.publish_pose()

    def update_pose(self, key):
        roll, pitch, yaw = self.get_euler_from_quaternion(self.int_marker.pose.orientation)
        if key == 'w':
            self.int_marker.pose.position.x += self.speed
        elif key == 's':
            self.int_marker.pose.position.x -= self.speed
        elif key == 'a':
            self.int_marker.pose.position.y += self.speed
        elif key == 'd':
            self.int_marker.pose.position.y -= self.speed
        elif key == 'z':  
            self.int_marker.pose.position.z -= self.speed
        elif key == ' ':
            self.int_marker.pose.position.z += self.speed
        elif key == 'q':
            yaw += self.rotation_speed
        elif key == 'e':
            yaw -= self.rotation_speed
        elif key == 'j':
            roll -= self.rotation_speed
        elif key == 'l':
            roll += self.rotation_speed
        elif key == 'i':
            pitch += self.rotation_speed
        elif key == 'k':
            pitch -= self.rotation_speed
        
        self.int_marker.pose.orientation = self.get_quaternion_from_euler(roll, pitch, yaw)
        self.server.setPose(self.int_marker.name, self.int_marker.pose)
        self.server.applyChanges()
        self.publish_pose()

    def publish_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.int_marker.header.frame_id
        pose_stamped.pose = self.int_marker.pose
        self.pub.publish(pose_stamped)

    def get_euler_from_quaternion(self, q):
        quaternion = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(quaternion)
        return euler

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*quaternion)

    def run(self):
        rospy.loginfo("Simulator running. Press keys to control the marker.")
        rate = rospy.Rate(10)  # 10Hz

        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    key = sys.stdin.read(1)
                    self.update_pose(key)
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

if __name__ == "__main__":
    controller = AxisController()
    controller.run()
