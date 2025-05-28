import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

class SpiralGlobalPlanNode(Node):
    def __init__(self):
        super().__init__('spiral_global_plan_node')
        self.get_logger().info("Spiral Global Plan Node started.")

        # Parameters
        self.declare_parameter('spiral_resolution', 0.5)  # Spiral step size (meters)
        self.declare_parameter('spiral_max_radius', 10.0)  # Max radius of spiral (meters)
        self.declare_parameter('frame_id', 'map')  # Coordinate frame for the plan

        # Read parameters
        self.spiral_resolution = self.get_parameter('spiral_resolution').get_parameter_value().double_value
        self.spiral_max_radius = self.get_parameter('spiral_max_radius').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers and Subscribers
        self.plan_publisher = self.create_publisher(Path, 'global_plan', 10)
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)

        # Internal state
        self.costmap = None

    def costmap_callback(self, msg):
        """Callback to update the costmap."""
        self.costmap = msg
        self.get_logger().info("Received costmap.")
        self.generate_and_publish_plan()

    def generate_and_publish_plan(self):
        """Generate a spiral global plan and publish it."""
        if self.costmap is None:
            self.get_logger().warning("No costmap available. Skipping plan generation.")
            return

        # Get start position from costmap origin
        origin = self.costmap.info.origin
        start_x, start_y = origin.position.x, origin.position.y

        # Create a spiral plan
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        radius = self.spiral_resolution  # Start radius
        angle_step = math.radians(10)  # Angle step for spiral
        visited = set()  # To keep track of visited grid cells

        while radius < self.spiral_max_radius:
            angle = 0
            while angle < 2 * math.pi:
                # Compute the next position in the spiral
                x = start_x + radius * math.cos(angle)
                y = start_y + radius * math.sin(angle)

                # Convert world coordinates to grid indices
                grid_x, grid_y = self.world_to_grid(x, y)

                # Avoid revisiting and check for valid cell
                if (grid_x, grid_y) not in visited and self.is_valid_cell(grid_x, grid_y):
                    visited.add((grid_x, grid_y))

                    # Add pose to the path
                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.header.stamp = path.header.stamp
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.orientation.w = 1.0  # Default orientation
                    path.poses.append(pose)

                angle += angle_step
            radius += self.spiral_resolution

        self.get_logger().info(f"Generated spiral plan with {len(path.poses)} poses.")
        self.plan_publisher.publish(path)

    def is_valid_cell(self, grid_x, grid_y):
        """Check if the grid cell is free (not an obstacle)."""
        if grid_x < 0 or grid_y < 0 or grid_x >= self.costmap.info.width or grid_y >= self.costmap.info.height:
            return False

        # Convert grid indices to 1D array index
        index = grid_y * self.costmap.info.width + grid_x
        return self.costmap.data[index] < 50  # Threshold for obstacle cells

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid indices."""
        grid_x = int((wx - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        grid_y = int((wy - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        return grid_x, grid_y

def main(args=None):
    rclpy.init(args=args)
    node = SpiralGlobalPlanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
