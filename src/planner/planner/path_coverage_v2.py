import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from shapely.geometry import Polygon, Point
import numpy as np
import json
import tempfile
from math import atan2, pi
import tf2_ros
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from nav2_msgs.srv import GetPlan
from rclpy.action import ActionClient

from planner.marker_visualization_v2 import MarkerVisualization
from planner.libs.list_helper import *
from planner.libs.trapezoidal_coverage import calc_path as trapezoid_calc_path
from planner.libs.border_drive import border_calc_path
from ament_index_python.packages import get_package_share_directory

INSCRIBED_INFLATED_OBSTACLE = 253

class MapDrive(MarkerVisualization):
    def __init__(self):
        super().__init__()

        self.declare_parameter("robot_width", 0.3)
        self.declare_parameter("costmap_max_non_lethal", 70)
        self.declare_parameter("boustrophedon_decomposition", True)
        self.declare_parameter("border_drive", False)
        self.declare_parameter("base_frame", "base_link")

        self.robot_width = self.get_parameter("robot_width").value
        self.costmap_max_non_lethal = self.get_parameter("costmap_max_non_lethal").value
        self.boustrophedon_decomposition = self.get_parameter("boustrophedon_decomposition").value
        self.border_drive = self.get_parameter("border_drive").value
        self.base_frame = self.get_parameter("base_frame").value

        self.lClickPoints = []
        self.global_frame = "map"
        self.local_costmap = None
        self.global_costmap = None

        self.create_subscription(PointStamped, "/clicked_point", self.rviz_point_received, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.global_costmap_cb, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.local_costmap_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup()
        self.get_logger().info("MapDrive initialized")

    def global_costmap_cb(self, msg):
        self.global_costmap = msg

    def local_costmap_cb(self, msg):
        self.local_costmap = msg
        self.local_costmap_width = msg.info.width * msg.info.resolution
        self.local_costmap_height = msg.info.height * msg.info.resolution

    def rviz_point_received(self, point):
        if self.global_costmap is None or self.local_costmap is None:
            self.get_logger().error("No global or local costmap, doing nothing.")
            return

        self.lClickPoints.append(point)
        points = [(p.point.x, p.point.y) for p in self.lClickPoints]
        self.global_frame = point.header.frame_id

        if len(self.lClickPoints) > 2:
            if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
                raise ValueError("Inconsistent frame_id in clicked points.")

            points_x = [p.point.x for p in self.lClickPoints]
            points_y = [p.point.y for p in self.lClickPoints]
            avg_x_dist = list_avg_dist(points_x)
            avg_y_dist = list_avg_dist(points_y)
            dist_x_first_last = abs(points_x[0] - points_x[-1])
            dist_y_first_last = abs(points_y[0] - points_y[-1])

            if dist_x_first_last < avg_x_dist / 10.0 and dist_y_first_last < avg_y_dist / 10.0:
                self.get_logger().info(f"Creating polygon {str(points)}")
                poly = Polygon(points)
                self.visualize_area(points, close=True)
                if self.boustrophedon_decomposition:
                    self.do_boustrophedon(poly, self.global_costmap)
                else:
                    self.drive_polygon(poly)
                self.visualize_area(points, close=True, show=False)
                self.lClickPoints = []
                return

        self.visualize_area(points, close=False)

    def do_boustrophedon(self, poly: Polygon, costmap: OccupancyGrid):
        # Get bounds of the polygon
        (minx, miny, maxx, maxy) = poly.bounds
        self.get_logger().info("Converting costmap at x=%.2f..%.2f, y=%.2f %.2f for Boustrophedon Decomposition" % (minx, maxx, miny, maxy))

        # Convert world coordinates to costmap indices
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height

        minx = round((minx - origin_x) / resolution)
        maxx = round((maxx - origin_x) / resolution)
        miny = round((miny - origin_y) / resolution)
        maxy = round((maxy - origin_y) / resolution)

		# Check min/max limits
        if minx < 0: minx = 0
        if maxx > width: maxx = width
        if miny < 0: miny = 0
        if maxy > height: maxy = height

        # Generate 2D grid for decomposition
        rows = []
        for ix in range(int(minx), int(maxx)):
            column = []
            for iy in range(int(miny), int(maxy)):
                x = ix * resolution + origin_x
                y = iy * resolution + origin_y
                data = costmap.data[int(iy * width + ix)]

                if data == -1 or not poly.contains(Point([x, y])):
                    # Unknown or not inside polygon: Treat as obstacle
                    column.append(0)   # Obstacle
                elif data <= self.costmap_max_non_lethal:
                    # Freespace (non-lethal)
                    column.append(-1)  # Free space
                else:
                    # Obstacle
                    column.append(0)   # Lethal obstacle
            rows.append(column)

        # Run external decomposition script
        polygons = []
        with tempfile.NamedTemporaryFile(delete=False, mode='w') as ftmp:
            ftmp.write(json.dumps(rows))
            ftmp.flush()

            # Adjust path if needed
            boustrophedon_script = os.path.join("/home", "robot", "sweepy_dev_ws", "src", "planner", "planner", "boustrophedon_decomposition.rb")
            with os.popen(f"{boustrophedon_script} {ftmp.name}") as fscript:
                polygons = json.loads(fscript.readline())

        # Convert decomposed cells back to world coordinates and drive them
        for poly in polygons:
            points = [
                (
                    (pt[0] + minx) * resolution + origin_x,
                    (pt[1] + miny) * resolution + origin_y
                ) for pt in poly]
            self.get_logger().debug("Creating polygon from Boustrophedon Decomposition %s" % str(points))
            self.drive_polygon(Polygon(points))

        self.get_logger().info("Boustrophedon Decomposition done")

    def build_pose_stamped(self, x, y, theta):
        q = quaternion_from_euler(0, 0, theta)
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    
    def drive_path(self, path):
        self.visualize_path(path)
        initial_pose = self.navigator.initial_pose
        if initial_pose is None:
            self.get_logger().warn("No initial pose set in navigator, using (0,0)")
            initial_pos = (0.0, 0.0)
        else:
            initial_pos = (initial_pose.pose.position.x, initial_pose.pose.position.y)
        path.insert(0, initial_pos)

        for pos_last, pos_next in pairwise(path):
            if not rclpy.ok():
                return
            pos_diff = np.array(pos_next) - np.array(pos_last)
            angle = atan2(pos_diff[1], pos_diff[0])

            rotate_pose = self.build_pose_stamped(pos_last[0], pos_last[1], angle)
            goal_pose = self.build_pose_stamped(pos_next[0], pos_next[1], angle)

            self.navigator.goToPose(rotate_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self)
            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f"Rotation goal failed at {pos_last}")
                continue

            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self)
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Successfully reached {pos_next}")
            else:
                self.get_logger().warn(f"Failed to reach {pos_next}")
        self.visualize_path(path, show=False)
        
    def drive_polygon(self, polygon):
        self.visualize_cell(polygon.exterior.coords[:])

        # Align longest side of the polygon to the horizontal axis
        angle = get_angle_of_longest_side_to_horizontal(polygon)
        if angle is None:
            self.get_logger().warn("Cannot align polygon.")
            return
        angle += pi/2 # up/down instead of left/right
        poly_rot = rotate_polygon(polygon, angle)
        self.get_logger().debug("Rotated polygon by %.0f°: %s" % (angle * 180 / pi, str(poly_rot.exterior.coords[:])))

        if self.border_drive:
            path_rot = border_calc_path(poly_rot, self.robot_width)
            path = rotate_points(path_rot, -angle)
            self.drive_path(path)

        # run
        path_rot = trapezoid_calc_path(poly_rot, self.robot_width)
        path = rotate_points(path_rot, -angle)
        self.drive_path(path)

        # cleanup
        self.visualize_cell(polygon.exterior.coords[:], show=False)
        self.get_logger().debug("Polygon done")

def main(args=None):
    rclpy.init(args=args)
    node = MapDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.visualization_cleanup()
        node.destroy_node()
        rclpy.shutdown()
