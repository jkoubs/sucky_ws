#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
import json
import tempfile
from rclpy.action import ActionClient
from shapely.geometry import Polygon, Point
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from math import *
from nav2_msgs.action import NavigateToPose
import tf2_ros
from planner.libs.border_drive import border_calc_path
from planner.libs.list_helper import *
from planner.marker_visualization import MarkerVisualization
from planner.libs.trapezoidal_coverage import calc_path as trapezoid_calc_path
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

INSCRIBED_INFLATED_OBSTACLE = 253

class MapDrive(Node):
    def __init__(self):
        super().__init__('path_coverage')
        self.get_logger().info("MapDrive node created")
        self.visualizer = MarkerVisualization(self)
        self.navigator = BasicNavigator()
        self.lClickPoints = []
        self.global_frame = "map"  # Default frame
        self.local_costmap = None
        self.global_costmap = None
        self.robot_width = self.declare_parameter("robot_width", 1.066).value
        self.costmap_max_non_lethal = self.declare_parameter("costmap_max_non_lethal", 70).value
        self.boustrophedon_decomposition = self.declare_parameter("boustrophedon_decomposition", True).value
        self.border_drive = self.declare_parameter("border_drive", False).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        self.create_subscription(PointStamped, "/clicked_point", self.rvizPointReceived, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.globalCostmapReceived, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.localCostmapReceived, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        try:
            self.navigator.waitUntilNav2Active()
            self.get_logger().info("Running...")
        except Exception as e:
            self.get_logger().error(f"Failed while waiting for Nav2: {e}")

        # self.navigator.waitUntilNav2Active()
        # self.get_logger().info("Running...")

    def globalCostmapReceived(self, costmap):
        self.global_costmap = costmap

    def localCostmapReceived(self, costmap):
        self.local_costmap = costmap
        self.local_costmap_width = costmap.info.width * costmap.info.resolution
        self.local_costmap_height = costmap.info.height * costmap.info.resolution

    def rvizPointReceived(self, point):
        if self.global_costmap is None or self.local_costmap is None:
            self.get_logger().error("No global or local costmap, doing nothing.")
            return
        self.lClickPoints.append(point)
        points = [(p.point.x, p.point.y) for p in self.lClickPoints]
        self.global_frame = point.header.frame_id
        if len(self.lClickPoints) > 2:
            # All points must have same frame_id
            if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
                raise ValueError()
            points_x = [p.point.x for p in self.lClickPoints]
            points_y = [p.point.y for p in self.lClickPoints]
            avg_x_dist = list_avg_dist(points_x)
            avg_y_dist = list_avg_dist(points_y)
            dist_x_first_last = abs(points_x[0] - points_x[-1])
            dist_y_first_last = abs(points_y[0] - points_y[-1])
            if dist_x_first_last < avg_x_dist / 10.0 and dist_y_first_last < avg_y_dist / 10.0:
                # last point is close to maximum, construct polygon
                self.get_logger().info(f"Creating polygon {points}")
                self.visualizer.visualize_area(points, close=True)
                if self.boustrophedon_decomposition:
                    self.do_boustrophedon(Polygon(points), self.global_costmap)
                else:
                    self.drive_polygon(Polygon(points))
                self.visualizer.visualize_area(points, close=True, show=False)
                self.lClickPoints = []
                return
        self.visualizer.visualize_area(points, close=False)

    def do_boustrophedon(self, poly, costmap):
        # Cut polygon area from costmap
        (minx, miny, maxx, maxy) = poly.bounds
        self.get_logger().info(f"Converting costmap at x={minx:.2f}..{maxx:.2f}, y={miny:.2f} {maxy:.2f} for Boustrophedon Decomposition")

        # Convert to costmap coordinate
        minx = round((minx - costmap.info.origin.position.x) / costmap.info.resolution)
        maxx = round((maxx - costmap.info.origin.position.x) / costmap.info.resolution)
        miny = round((miny - costmap.info.origin.position.y) / costmap.info.resolution)
        maxy = round((maxy - costmap.info.origin.position.y) / costmap.info.resolution)

        # Check min/max limits
        if minx < 0: minx = 0
        if maxx > costmap.info.width: maxx = costmap.info.width
        if miny < 0: miny = 0
        if maxy > costmap.info.height: maxy = costmap.info.height

        # Transform costmap values to values expected by boustrophedon_decomposition script
        rows = []
        for ix in range(int(minx), int(maxx)):
            column = []
            for iy in range(int(miny), int(maxy)):
                x = ix * costmap.info.resolution + costmap.info.origin.position.x
                y = iy * costmap.info.resolution + costmap.info.origin.position.y
                data = costmap.data[int(iy * costmap.info.width + ix)]
                if data == -1 or not poly.contains(Point([x, y])):
                    # Unknown or not inside polygon: Treat as obstacle
                    column.append(0)
                elif data <= self.costmap_max_non_lethal:
                    # Freespace (non-lethal)
                    column.append(-1)
                else:
                    # Obstacle
                    column.append(0)
            rows.append(column)
        polygons = []
        with tempfile.NamedTemporaryFile(delete=False, mode='w') as ftmp:
            ftmp.write(json.dumps(rows))
            ftmp.flush()
            boustrophedon_script = os.path.join("/home", "robot", "sweepy_dev_ws", "src", "planner", "planner", "boustrophedon_decomposition.rb")
            with os.popen("%s %s" % (boustrophedon_script, ftmp.name)) as fscript:
                polygons = json.loads(fscript.readline())

        for poly in polygons:
            points = [
                (
                    (point[0] + minx) * costmap.info.resolution + costmap.info.origin.position.x,
                    (point[1] + miny) * costmap.info.resolution + costmap.info.origin.position.y
                ) for point in poly]
            self.get_logger().debug(f"Creating polygon from Boustrophedon Decomposition {points}")
            self.drive_polygon(Polygon(points))
        self.get_logger().info("Boustrophedon Decomposition done")

    def next_pos(self, x, y, angle):
        self.get_logger().info(f"Moving to ({x}, {y}, {angle * 180 / pi:.0f}°)")

        goal_msg = PoseStamped()
        angle_quat = tf_transformations.quaternion_from_euler(0, 0, angle)
        goal_msg.header.frame_id = self.global_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.x = angle_quat[0]
        goal_msg.pose.orientation.y = angle_quat[1]
        goal_msg.pose.orientation.z = angle_quat[2]
        goal_msg.pose.orientation.w = angle_quat[3]
        
        self.navigator.goToPose(goal_msg)
        self.get_logger().info("Send Goal")
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            print(feedback)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.current_behavior}")
    
    def get_closes_possible_goal(self, pos_last, pos_next, angle, tolerance):
        # Placeholder for compatibility
        return pos_next

    def drive_path(self, path):
        self.visualizer.visualize_path(path)

        initial_pos = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        path.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))

        for pos_last, pos_next in zip(path[:-1], path[1:]):
            if not rclpy.ok(): return

            pos_diff = np.array(pos_next) - np.array(pos_last)
            # angle from last to current position
            angle = atan2(pos_diff[1], pos_diff[0])

            if abs(pos_diff[0]) < self.local_costmap_width / 2.0 and abs(pos_diff[1]) < self.local_costmap_height / 2.0:
                # goal is visible in local costmap, check path is clear
                closest = self.get_closes_possible_goal(pos_last, pos_next, angle, min(pos_diff[0], pos_diff[1]))
                if closest is None:
                    continue
                pos_next = closest

            self.next_pos(pos_last[0], pos_last[1], angle)  # rotate in direction of next goal
            self.next_pos(pos_next[0], pos_next[1], angle)
        self.visualizer.visualize_path(path, False)

    def drive_polygon(self, polygon):
        self.visualizer.visualize_cell(polygon.exterior.coords[:])

        # Align longest side of the polygon to the horizontal axis
        angle = get_angle_of_longest_side_to_horizontal(polygon)
        if angle is None:
            self.get_logger().warn("Can not return polygon")
            return
        angle += pi / 2  # up/down instead of left/right
        poly_rotated = rotate_polygon(polygon, angle)
        self.get_logger().debug(f"Rotated polygon by {angle * 180 / pi:.0f}°: {poly_rotated.exterior.coords[:]}")

        if self.border_drive:
            path_rotated = border_calc_path(poly_rotated, self.robot_width)
            path = rotate_points(path_rotated, -angle)
            self.drive_path(path)

        # run
        path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width)
        path = rotate_points(path_rotated, -angle)
        self.drive_path(path)

        # cleanup
        self.visualizer.visualize_cell(polygon.exterior.coords[:], False)
        self.get_logger().debug("Polygon done")


def main(args=None):
    if rclpy.ok():
        raise RuntimeError("rclpy is already initialized! Are you creating a second node?")
    rclpy.init(args=args)
    node = MapDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
