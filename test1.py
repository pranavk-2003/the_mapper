import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


class PCDToOccupancyGridPublisher(Node):
    def __init__(self, pcd_file, resolution=0.05, grid_size=(200, 200)):
        super().__init__('pcd_to_occupancy_grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        self.pcd_file = pcd_file
        self.resolution = resolution
        self.grid_size = grid_size

        # Load the .pcd file and generate the occupancy grid
        self.occupancy_grid = self.pcd_to_occupancy_grid()

        # Publish the grid at regular intervals
        self.timer = self.create_timer(1.0, self.publish_occupancy_grid)

    def pcd_to_occupancy_grid(self):
        """
        Converts a .pcd file to a 2D occupancy grid.
        """
        # Load the point cloud
        pcd = o3d.io.read_point_cloud(self.pcd_file)
        points = np.asarray(pcd.points)

        # Project points onto the XY plane
        xy_points = points[:, :2]

        # Initialize the grid
        width, height = self.grid_size
        grid = np.zeros((height, width), dtype=np.int8)

        # Determine bounds of the point cloud
        min_x, min_y = xy_points.min(axis=0)
        max_x, max_y = xy_points.max(axis=0)

        x_range = max_x - min_x
        y_range = max_y - min_y

        # Map points to grid
        for point in xy_points:
            x, y = point
            grid_x = int((x - min_x) / x_range * (width - 1))
            grid_y = int((y - min_y) / y_range * (height - 1))
            grid[grid_y, grid_x] = 100  # Occupied cell

        return grid

    def publish_occupancy_grid(self):
        """
        Publishes the 2D occupancy grid as a ROS OccupancyGrid message.
        """
        grid = self.occupancy_grid
        resolution = self.resolution
        height, width = grid.shape

        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'

        grid_msg.info.resolution = resolution
        grid_msg.info.width = width
        grid_msg.info.height = height

        # Set the origin of the map
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0
        grid_msg.info.origin.position.z = 0.0

        # Flatten the grid and assign it to the data field
        grid_msg.data = grid.flatten().tolist()

        # Publish the occupancy grid
        self.get_logger().info('Publishing occupancy grid...')
        self.publisher_.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)

    # Path to your .pcd file
    pcd_file_path = '/home/pranav/the_mapper/octomap_point_cloud_centers.pcd'
    # Parameters
    resolution = 0.05  # Grid resolution in meters per cell
    grid_size = (200, 200)  # Grid size (width, height)

    # Create and spin the node
    node = PCDToOccupancyGridPublisher(pcd_file_path, resolution, grid_size)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
