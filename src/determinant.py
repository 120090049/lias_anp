#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import numpy as np

class InteractiveMarkerExample:
    def __init__(self):
        rospy.init_node("interactive_marker_example")
        self.server = InteractiveMarkerServer("interactive_marker_example")

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "map"
        self.int_marker.name = "my_marker"
        self.int_marker.description = "Simple 6-DOF Control"
        self.int_marker.pose.position = Point(0, 0, 0)

        self.add_controls()

        self.server.insert(self.int_marker, self.process_feedback)
        self.server.applyChanges()

        self.pose_history = []
        self.history_index = -1

        rospy.Subscriber("/interactive_marker_example/feedback", InteractiveMarkerFeedback, self.feedback_callback)

        rospy.spin()

    def add_controls(self):
        # Create controls for moving and rotating the marker along and around the X, Y, Z axes
        axes = ['x', 'y', 'z']
        for axis in axes:
            self.add_move_control(axis)
            self.add_rotate_control(axis)

    def add_move_control(self, axis):
        control = InteractiveMarkerControl()
        control.name = f"move_{axis}"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.set_control_orientation(control, axis)
        self.int_marker.controls.append(control)

    def add_rotate_control(self, axis):
        control = InteractiveMarkerControl()
        control.name = f"rotate_{axis}"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.set_control_orientation(control, axis)
        self.int_marker.controls.append(control)

    def set_control_orientation(self, control, axis):
        control.orientation.w = 1
        control.orientation.x = 1 if axis == 'x' else 0
        control.orientation.y = 1 if axis == 'y' else 0
        control.orientation.z = 1 if axis == 'z' else 0

    def process_feedback(self, feedback):
        # Record pose history on movement feedback
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.record_pose(feedback.pose)

    def feedback_callback(self, feedback):
        if feedback.marker_name == self.int_marker.name:
            self.process_feedback(feedback)

    def record_pose(self, pose):
        # Remove any "redo" history if we are recording a new pose
        if self.history_index < len(self.pose_history) - 1:
            self.pose_history = self.pose_history[:self.history_index + 1]
        self.pose_history.append(pose)
        self.history_index += 1

    def undo(self):
        if self.history_index > 0:
            self.history_index -= 1
            self.int_marker.pose = self.pose_history[self.history_index]
            self.server.insert(self.int_marker)
            self.server.applyChanges()

    def redo(self):
        if self.history_index < len(self.pose_history) - 1:
            self.history_index += 1
            self.int_marker.pose = self.pose_history[self.history_index]
            self.server.insert(self.int_marker)
            self.server.applyChanges()

    def handle_key_event(self):
        try:
            if rospy.get_param('/interactive_marker_example/undo'):
                self.undo()
                rospy.set_param('/interactive_marker_example/undo', False)
            elif rospy.get_param('/interactive_marker_example/redo'):
                self.redo()
                rospy.set_param('/interactive_marker_example/redo', False)
        except KeyError:
            pass

if __name__ == "__main__":
    node = InteractiveMarkerExample()

    # Use a timer to periodically check for key events
    rospy.Timer(rospy.Duration(0.1), lambda event: node.handle_key_event())
    rospy.spin()
