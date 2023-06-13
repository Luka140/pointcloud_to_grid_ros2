import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import pcl
# import pcl_conversions

class PointXY:
    def __init__(self):
        self.x = 0
        self.y = 0

class PointXYZI:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.intensity = 0.0

class GridMap:
    def __init__(self):
        self.position_x = 0.0
        self.position_y = 0.0
        self.cell_size = 0.0
        self.length_x = 0.0
        self.length_y = 0.0
        self.cloud_in_topic = ""
        self.frame_out = ""
        self.mapi_topic_name = ""
        self.maph_topic_name = ""
        self.topleft_x = 0.0
        self.topleft_y = 0.0
        self.bottomright_x = 0.0
        self.bottomright_y = 0.0
        self.cell_num_x = 0
        self.cell_num_y = 0
        self.intensity_factor = 0.0
        self.height_factor = 0.0

    def initGrid(self, grid : OccupancyGrid):
        grid.header.frame_id = self.frame_out  # TODO
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 1.0
        grid.info.origin.position.x = self.position_x + self.length_x / 2.0
        grid.info.origin.position.y = self.position_y + self.length_y / 2.0
        grid.info.width = int(self.length_x / self.cell_size)
        grid.info.height = int(self.length_y / self.cell_size)
        grid.info.resolution = self.cell_size

    def paramRefresh(self):
        self.topleft_x = self.position_x + self.length_x / 2.0
        self.bottomright_x = self.position_x - self.length_x / 2.0
        self.topleft_y = self.position_y + self.length_y / 2.0
        self.bottomright_y = self.position_y - self.length_y / 2.0
        self.cell_num_x = int(self.length_x / self.cell_size)
        self.cell_num_y = int(self.length_y / self.cell_size)
        if self.cell_num_x > 0:
            print(
                "Cells: %d*%d px, subscribed to %s [%f, %f] [%f, %f]"
                % (
                    self.cell_num_x,
                    self.cell_num_y,
                    self.cloud_in_topic,
                    self.topleft_x,
                    self.topleft_y,
                    self.bottomright_x,
                    self.bottomright_y,
                )
            )

    def getSize(self):
        return self.cell_num_x * self.cell_num_y

    def getSizeX(self):
        return self.cell_num_x

    def getSizeY(self):
        return self.cell_num_y

    def getLengthX(self):
        return self.length_x

    def getLengthY(self):
        return self.length_y

    def getResolution(self):
        return self.cell_size


class PointCloudToGridNode(Node):
    def __init__(self):
        super().__init__("pointcloud_to_grid_node")
        self.grid_map = GridMap()
        self.pub_igrid = self.create_publisher(
            OccupancyGrid, self.grid_map.mapi_topic_name, 1
        )
        self.pub_hgrid = self.create_publisher(
            OccupancyGrid, self.grid_map.maph_topic_name, 1
        )
        self.sub_pc2 = self.create_subscription(
            PointCloud2, self.grid_map.cloud_in_topic, self.pointcloudCallback, 1
        )
        self.intensity_grid = OccupancyGrid()
        self.height_grid = OccupancyGrid()

    def getIndex(self, x, y):
        ret = PointXY()
        ret.x = int(abs(x - self.grid_map.topleft_x) / self.grid_map.cell_size)
        ret.y = int(abs(y - self.grid_map.topleft_y) / self.grid_map.cell_size)
        return ret

    def paramsCallback(self, config, level):
        self.grid_map.cell_size = config.cell_size
        self.grid_map.position_x = config.position_x
        self.grid_map.position_y = config.position_y
        self.grid_map.cell_size = config.cell_size
        self.grid_map.length_x = config.length_x
        self.grid_map.length_y = config.length_y
        self.grid_map.cloud_in_topic = config.cloud_in_topic
        self.grid_map.intensity_factor = config.intensity_factor
        self.grid_map.height_factor = config.height_factor
        self.grid_map.mapi_topic_name = config.mapi_topic_name
        self.grid_map.maph_topic_name = config.maph_topic_name
        self.grid_map.cloud_in_topic = config.cloud_in_topic
        self.grid_map.initGrid(self.intensity_grid)
        self.grid_map.initGrid(self.height_grid)
        self.grid_map.paramRefresh()

    def pointcloudCallback(self, msg):
        # out_cloud = pcl.PointCloud.PointXYZI()
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)
        # rclpy.pcl_conversions.fromPCL(msg, out_cloud)

        self.grid_map.initGrid(self.intensity_grid)
        self.grid_map.initGrid(self.height_grid)

        hpoints = [0] * (self.grid_map.cell_num_x * self.grid_map.cell_num_y)
        ipoints = [0] * (self.grid_map.cell_num_x * self.grid_map.cell_num_y)

        for out_point in out_cloud:
            if out_point.x > 0.01 or out_point.x < -0.01:
                if (
                    out_point.x > self.grid_map.bottomright_x
                    and out_point.x < self.grid_map.topleft_x
                ):
                    if (
                        out_point.y > self.grid_map.bottomright_y
                        and out_point.y < self.grid_map.topleft_y
                    ):
                        cell = self.getIndex(out_point.x, out_point.y)
                        if (
                            cell.x < self.grid_map.cell_num_x
                            and cell.y < self.grid_map.cell_num_y
                        ):
                            ipoints[cell.y * self.grid_map.cell_num_x + cell.x] = (
                                out_point.intensity * self.grid_map.intensity_factor
                            )
                            hpoints[cell.y * self.grid_map.cell_num_x + cell.x] = (
                                out_point.z * self.grid_map.height_factor
                            )
                        else:
                            self.get_logger().warn(
                                "Cell out of range: %s - %s ||| %s - %s"
                                % (
                                    cell.x,
                                    self.grid_map.cell_num_x,
                                    cell.y,
                                    self.grid_map.cell_num_y,
                                )
                            )

        self.intensity_grid.header.stamp = self.get_clock().now().to_msg()
        self.intensity_grid.header.frame_id = msg.header.frame_id  # TODO
        self.intensity_grid.info.map_load_time = self.get_clock().now().to_msg()
        self.intensity_grid.data = ipoints

        self.height_grid.header.stamp = self.get_clock().now().to_msg()
        self.height_grid.header.frame_id = msg.header.frame_id  # TODO
        self.height_grid.info.map_load_time = self.get_clock().now().to_msg()
        self.height_grid.data = hpoints

        self.pub_igrid.publish(self.intensity_grid)
        self.pub_hgrid.publish(self.height_grid)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
