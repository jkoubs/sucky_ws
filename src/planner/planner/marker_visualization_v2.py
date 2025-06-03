import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from planner.libs.list_helper import pairwise  # Adjust import if needed

class MarkerVisualization(Node):
    def __init__(self):
        super().__init__('marker_visualization')
        self.last_points = {}
        self.last_path = None

        self.declare_parameter('global_frame', 'map')
        self.global_frame = self.get_parameter('global_frame').value

        self.pub_marker = self.create_publisher(Marker, 'path_coverage_marker', 16)

    def visualization_cleanup(self):
        for id, points in self.last_points.items():
            if points is not None:
                self.visualize_trapezoid(points, id=id, show=False)
            self.last_points = {}

        if self.last_path is not None:
            self.visualize_path(self.last_path, show=False)
            self.last_path = None

    def visualize_cell(self, points, show=True, close=True):
        self.visualize_trapezoid(points, show, close)

    def visualize_area(self, points, show=True, close=True):
        self.visualize_trapezoid(points, show, close, id=1, red=1.0, blue=0.0)

    def visualize_trapezoid(self, points, show=True, close=True, id=0, red=0.0, green=0.0, blue=1.0):
        if len(points) < 2:
            return

        self.last_points[id] = points if show else None

        msg = Marker()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "trapezoid"
        msg.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        msg.id = id
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD if show else Marker.DELETE
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.scale.x = 0.02
        msg.color.r = red
        msg.color.g = green
        msg.color.b = blue
        msg.color.a = 1.0

        if close:
            points = points + [points[0]]

        for point in points:
            pt = Point()
            pt.x = point[0]
            pt.y = point[1]
            msg.points.append(pt)

        self.pub_marker.publish(msg)

    def visualize_path(self, path, show=True):
        i = 0
        self.last_path = path if show else None
        for pos_last, pos_cur in pairwise(path):
            msg = Marker()
            msg.header.frame_id = self.global_frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.ns = "path"
            msg.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            msg.id = i
            msg.type = Marker.ARROW
            msg.action = Marker.ADD if show else Marker.DELETE
            
            # Full orientation set to identity
            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0

            # Scale (shaft & head)
            msg.scale.x = 0.03  # shaft diameter
            msg.scale.y = 0.07  # head diameter

            # Color (green)
            msg.color.r = 0.0
            msg.color.g = 1.0
            msg.color.b = 0.0
            msg.color.a = 1.0

            # Add start and end points
            pt_start = Point()
            pt_start.x = pos_last[0]
            pt_start.y = pos_last[1]
            msg.points.append(pt_start)

            pt_end = Point()
            pt_end.x = pos_cur[0]
            pt_end.y = pos_cur[1]
            msg.points.append(pt_end)

            i += 1
            self.pub_marker.publish(msg)