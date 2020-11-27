#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from runbot_routing_machine.msg import OutputCoords
from geometry_msgs.msg import Point

class GlobalPathPlannerNode:
    def __init__(self):
        rospy.init_node("routing_machine_route_handler", anonymous=True)

        # ROS publishers
        self.path_publisher_as_line_strip = rospy.Publisher('runbot_routing_machine/global_path', MarkerArray, queue_size=10)

        # ROS subscribers
        rospy.Subscriber("/runbot_routing_machine/global_waypoints", OutputCoords, self.callback_global_path, queue_size=10)

    def callback_global_path(self, coords):
        path = []
        for coord in coords.utm_coordinates:
            path += [[coord.x, coord.y]]
        if path: print (path)
        self.publish_path_as_line_strip(path)

    def publish_path_as_line_strip(self, path):
        markers = []
        i = 0
        path = np.array([path])
        for element in path:
            i += 1
            marker = Marker()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.ns = "path"
            marker.scale.x = 6
            marker.scale.y = 6
            marker.scale.z = 6
            marker.color.a = 1
            marker.color.r = 100/255
            marker.color.g = 100/255
            marker.color.b = 1
            marker.id = i
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.
            for point_ in element:
                point = Point()
                point.x = point_[0]
                point.y = point_[1]
                point.z = 0.
                marker.points.append(point)
            markers.append(marker)
        self.path_publisher_as_line_strip.publish(markers)

if __name__ == "__main__":
    planner = GlobalPathPlannerNode()
    rospy.spin()



