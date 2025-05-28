#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
import numpy as np
import json
import tempfile
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf2_geometry_msgs
from libs.list_helper import *
from libs.marker_visualization import MarkerVisualization
from shapely.geometry import Polygon, Point
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import GetPlan
from math import *
from libs.trapezoidal_coverage import calc_path as trapezoid_calc_path
from libs.border_drive import border_calc_path


INSCRIBED_INFLATED_OBSTACLE = 253

class MapDrive(Node, MarkerVisualization):
    def __init__(self):
        super().__init__('map_drive')
        MarkerVisualization.__init__(self)

        self.lClickPoints = []
        self.global_frame = "map"  # read from "/clicked_point"
        self.local_costmap = None
        self.global_costmap = None
        
        # Get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_width', 0.3),
                ('costmap_max_non_lethal', 70),
                ('boustrophedon_decomposition', True),
                ('border_drive', False),
                ('base_frame', 'base_link')
            ]
        )
        
        self.robot_width = self.get_parameter('robot_width').value
        self.costmap_max_non_lethal = self.get_parameter('costmap_max_non_lethal').value
        self.boustrophedon_decomposition = self.get_parameter('boustrophedon_decomposition').value
        self.border_drive = self.get_parameter('border_drive').value
        self.base_frame = self.get_parameter('base_frame').value

        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Set up subscribers
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.rvizPointReceived,
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.globalCostmapReceived,
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.localCostmapReceived,
            10,
            callback_group=self.callback_group
        )

        # Set up transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set up action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Set up service client for path planning
        self.move_base_plan = self.create_client(
            GetPlan,
            '/planner_server/get_plan',
            callback_group=self.callback_group
        )

        self.get_logger().info('Waiting for the navigation action server to come up')
        self.nav_client.wait_for_server()
        self.get_logger().info('Got navigation action server')

        self.get_logger().info('Running..')

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
            if dist_x_first_last < avg_x_dist/10.0 and dist_y_first_last < avg_y_dist/10.0:
                # last point is close to maximum, construct polygon
                self.get_logger().info(f"Creating polygon {str(points)}")
                self.visualize_area(points, close=True)
                if self.boustrophedon_decomposition:
                    self.do_boustrophedon(Polygon(points), self.global_costmap)
                else:
                    self.drive_polygon(Polygon(points))
                self.visualize_area(points, close=True, show=False)
                self.lClickPoints = []
                return
        self.visualize_area(points, close=False)

    async def next_pos(self, x, y, angle):
        self.get_logger().info(f"Moving to ({x}, {y}, {angle*180/pi}°)")

        goal_msg = NavigateToPose.Goal()
        angle_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, angle)
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = angle_quat[0]
        goal_msg.pose.pose.orientation.y = angle_quat[1]
        goal_msg.pose.pose.orientation.z = angle_quat[2]
        goal_msg.pose.pose.orientation.w = angle_quat[3]

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to ({x}, {y}) was rejected!")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == NavigateToPose.Result.STATUS_SUCCEEDED:
            self.get_logger().info(f"The base moved to ({x}, {y})")
        else:
            self.get_logger().error(f"The base failed moving to ({x}, {y})")

    def on_shutdown(self):
        self.get_logger().info("Canceling all goals")
        self.visualization_cleanup()
        # Cancel any active goals
        if self.nav_client.server_is_ready():
            for goal_handle in self.nav_client._goal_handles:
                goal_handle.cancel_goal_async()

    async def get_closes_possible_goal(self, pos_last, pos_next, angle, tolerance):
        angle_quat = tf2_ros.transformations.quaternion_from_euler(0, 0, angle)
        
        start = PoseStamped()
        start.header.frame_id = self.global_frame
        start.pose.position.x = pos_last[0]
        start.pose.position.y = pos_last[1]
        start.pose.orientation.x = angle_quat[0]
        start.pose.orientation.y = angle_quat[1]
        start.pose.orientation.z = angle_quat[2]
        start.pose.orientation.w = angle_quat[3]
        
        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.pose.position.x = pos_next[0]
        goal.pose.position.y = pos_next[1]
        goal.pose.orientation.x = angle_quat[0]
        goal.pose.orientation.y = angle_quat[1]
        goal.pose.orientation.z = angle_quat[2]
        goal.pose.orientation.w = angle_quat[3]

        request = GetPlan.Request()
        request.start = start
        request.goal = goal
        request.tolerance = tolerance

        future = self.move_base_plan.call_async(request)
        await rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            return None
            
        plan = future.result().plan
        if len(plan.poses) == 0:
            return None

        closest = None
        for pose in plan.poses:
            pose.header.stamp = self.get_clock().now().to_msg()

            try:
                local_pose = self.tf_buffer.transform(pose, self.local_costmap.header.frame_id)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Transform failed: {str(e)}")
                continue

            cellx = round((local_pose.pose.position.x - self.local_costmap.info.origin.position.x) / self.local_costmap.info.resolution)
            celly = round((local_pose.pose.position.y - self.local_costmap.info.origin.position.y) / self.local_costmap.info.resolution)
            cellidx = int(celly * self.local_costmap.info.width + cellx)
            
            if cellidx < 0 or cellidx >= len(self.local_costmap.data):
                self.get_logger().warn("get_closes_possible_goal landed outside costmap, returning original goal.")
                return pos_next
                
            cost = self.local_costmap.data[cellidx]

            if cost >= INSCRIBED_INFLATED_OBSTACLE:
                break

            closest = pose
            
        return (closest.pose.position.x, closest.pose.position.y) if closest else None

    async def drive_path(self, path):
        self.visualize_path(path)

        try:
            initial_pos = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            path.insert(0, (initial_pos.transform.translation.x, initial_pos.transform.translation.y))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')
            return

        for pos_last, pos_next in pairwise(path):
            if not rclpy.ok():
                return

            pos_diff = np.array(pos_next) - np.array(pos_last)
            angle = atan2(pos_diff[1], pos_diff[0])

            if abs(pos_diff[0]) < self.local_costmap_width/2.0 and abs(pos_diff[1]) < self.local_costmap_height/2.0:
                tolerance = min(pos_diff[0], pos_diff[1])
                closest = await self.get_closes_possible_goal(pos_last, pos_next, angle, tolerance)
                if closest is None:
                    continue
                pos_next = closest

            await self.next_pos(pos_last[0], pos_last[1], angle)
            await self.next_pos(pos_next[0], pos_next[1], angle)

        self.visualize_path(path, False)

    async def drive_polygon(self, polygon):
        self.visualize_cell(polygon.exterior.coords[:])

        angle = get_angle_of_longest_side_to_horizontal(polygon)
        if angle is None:
            self.get_logger().warn("Cannot return polygon")
            return
            
        angle += pi/2
        poly_rotated = rotate_polygon(polygon, angle)
        self.get_logger().debug(f"Rotated polygon by {angle*180/pi}°: {str(poly_rotated.exterior.coords[:])}")

        if self.border_drive:
            path_rotated = border_calc_path(poly_rotated, self.robot_width)
            path = rotate_points(path_rotated, -angle)
            await self.drive_path(path)

        path_rotated = trapezoid_calc_path(poly_rotated, self.robot_width)
        path = rotate_points(path_rotated, -angle)
        await self.drive_path(path)

        self.visualize_cell(polygon.exterior.coords[:], False)
        self.get_logger().debug("Polygon done")

def main(args=None):
    rclpy.init(args=args)
    node = MapDrive()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
